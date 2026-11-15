#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <traj_utils/MINCOTraj.h>
#include <quadrotor_msgs/GoalSet.h>
#include <sensor_msgs/Joy.h>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <fstream>
#include <sstream>
#include <deque>
#include <random>
#include <algorithm>
#include <iomanip>
#include <atomic>
#define UDP_PORT 8081
#define BUF_LEN 1048576    // 1MB
#define BUF_LEN_SHORT 1024 // 1KB

using namespace std;

bool ensure_dir_recursive(const string &path);
string json_escape(const string &s);
string now_iso8601();

int udp_server_fd_, udp_send_fd_;
ros::Subscriber other_odoms_sub_, one_traj_sub_, mandatory_stop_sub_, goal_sub_, joy_sub_;
ros::Subscriber comm_incident_sub_, comm_mandatory_stop_sub_;
ros::Publisher other_odoms_pub_, one_traj_pub_, mandatory_stop_pub_, goal_pub_, joy_pub_;
string udp_ip_;
int drone_id_;
double odom_broadcast_freq_;
char udp_recv_buf_[BUF_LEN], udp_send_buf_[BUF_LEN];
struct sockaddr_in addr_udp_send_;
nav_msgs::Odometry odom_msg_;
traj_utils::MINCOTraj MINCOTraj_msg_;
std_msgs::Empty stop_msg_;
quadrotor_msgs::GoalSet goal_msg_;
sensor_msgs::Joy joy_msg_;

std::atomic<bool> udp_recv_running_(false);

boost::mutex udp_send_mtx_;

bool comm_fuzz_enable_;
bool legacy_passthrough_when_no_fuzz_;
int comm_fuzz_target_drone_id_;
string comm_fuzz_mode_cfg_;
int comm_fuzz_seed_cfg_;
double comm_fuzz_start_min_s_, comm_fuzz_start_max_s_;
double comm_fuzz_duration_min_s_, comm_fuzz_duration_max_s_;
double comm_fuzz_delay_min_ms_, comm_fuzz_delay_max_ms_;
double comm_fuzz_jitter_min_ms_, comm_fuzz_jitter_max_ms_;
double comm_fuzz_noise_sigma_pos_min_, comm_fuzz_noise_sigma_pos_max_;
string comm_fuzz_report_dir_, comm_fuzz_run_id_;
bool comm_fuzz_trace_enable_;

string comm_fuzz_mode_used_, comm_fuzz_factor_name_;
double comm_fuzz_factor_value_;
double comm_fuzz_window_start_s_, comm_fuzz_window_duration_s_, comm_fuzz_window_end_s_;
double comm_fuzz_start_ros_time_;
int comm_fuzz_seed_used_;
unsigned long long comm_fuzz_msg_seen_, comm_fuzz_msg_fuzzed_;

boost::mutex comm_fuzz_meta_mtx_;
std::mt19937 comm_fuzz_rng_;
boost::mutex comm_fuzz_trace_mtx_;
unsigned long long comm_fuzz_trace_seq_;

struct ScheduledTraj
{
  traj_utils::MINCOTraj msg;
  ros::WallTime due;
  ros::WallTime enqueued;
  double planned_delay_s;
  unsigned long long trace_id;
  string trace_event;
};

boost::mutex comm_fuzz_delay_mtx_;
boost::condition_variable comm_fuzz_delay_cv_;
std::deque<ScheduledTraj> comm_fuzz_delay_queue_;
bool comm_fuzz_delay_worker_running_;

string comm_fuzz_start_wall_time_;

bool comm_full_dump_enable_;
int comm_full_dump_buffer_max_;
string comm_full_dump_file_name_;
bool comm_full_dump_triggered_;
string comm_full_dump_trigger_reason_;
double comm_full_dump_trigger_ros_time_;
unsigned long long comm_full_dump_buffered_count_;
unsigned long long comm_full_dump_written_count_;
unsigned long long comm_full_dump_drop_count_;
boost::mutex comm_full_dump_mtx_;
std::deque<string> comm_full_dump_buffer_;

string comm_fuzz_run_dir()
{
  string run_id = comm_fuzz_run_id_;
  if (run_id.empty())
    run_id = "bridge_fuzz";
  return comm_fuzz_report_dir_ + "/" + run_id;
}

template <typename Arr>
string minco_array_to_json(const Arr &arr)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);
  oss << "[";
  size_t idx = 0;
  for (typename Arr::const_iterator it = arr.begin(); it != arr.end(); ++it, ++idx)
  {
    if (idx > 0)
      oss << ",";
    oss << static_cast<double>(*it);
  }
  oss << "]";
  return oss.str();
}

string comm_full_dump_build_line(const traj_utils::MINCOTraj &msg,
                                 const string &direction,
                                 const string &event,
                                 const string &note)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);
  oss << "{";
  oss << "\"wall_time\":\"" << json_escape(now_iso8601()) << "\",";
  oss << "\"ros_time\":" << ros::Time::now().toSec() << ",";
  oss << "\"bridge_drone_id\":" << drone_id_ << ",";
  oss << "\"direction\":\"" << json_escape(direction) << "\",";
  oss << "\"event\":\"" << json_escape(event) << "\",";
  oss << "\"note\":\"" << json_escape(note) << "\",";
  oss << "\"drone_id\":" << msg.drone_id << ",";
  oss << "\"traj_id\":" << msg.traj_id << ",";
  oss << "\"start_time\":" << msg.start_time.toSec() << ",";
  oss << "\"des_clearance\":" << static_cast<double>(msg.des_clearance) << ",";
  oss << "\"order\":" << static_cast<int>(msg.order) << ",";
  oss << "\"start_p\":" << minco_array_to_json(msg.start_p) << ",";
  oss << "\"start_v\":" << minco_array_to_json(msg.start_v) << ",";
  oss << "\"start_a\":" << minco_array_to_json(msg.start_a) << ",";
  oss << "\"end_p\":" << minco_array_to_json(msg.end_p) << ",";
  oss << "\"end_v\":" << minco_array_to_json(msg.end_v) << ",";
  oss << "\"end_a\":" << minco_array_to_json(msg.end_a) << ",";
  oss << "\"inner_x\":" << minco_array_to_json(msg.inner_x) << ",";
  oss << "\"inner_y\":" << minco_array_to_json(msg.inner_y) << ",";
  oss << "\"inner_z\":" << minco_array_to_json(msg.inner_z) << ",";
  oss << "\"duration\":" << minco_array_to_json(msg.duration);
  oss << "}";
  return oss.str();
}

void comm_full_dump_append(const traj_utils::MINCOTraj &msg,
                           const string &direction,
                           const string &event,
                           const string &note)
{
  if (!comm_full_dump_enable_)
    return;

  const string line = comm_full_dump_build_line(msg, direction, event, note);
  boost::lock_guard<boost::mutex> lock(comm_full_dump_mtx_);
  if (comm_full_dump_triggered_)
  {
    const string path = comm_fuzz_run_dir() + "/" + comm_full_dump_file_name_;
    std::ofstream ofs(path.c_str(), std::ios::out | std::ios::app);
    if (!ofs.is_open())
      return;
    ofs << line << "\n";
    comm_full_dump_written_count_++;
    return;
  }

  comm_full_dump_buffer_.push_back(line);
  comm_full_dump_buffered_count_++;
  while (static_cast<int>(comm_full_dump_buffer_.size()) > std::max(1, comm_full_dump_buffer_max_))
  {
    comm_full_dump_buffer_.pop_front();
    comm_full_dump_drop_count_++;
  }
}

void comm_full_dump_trigger(const string &reason)
{
  if (!comm_full_dump_enable_)
    return;

  boost::lock_guard<boost::mutex> lock(comm_full_dump_mtx_);
  if (comm_full_dump_triggered_)
    return;

  const string run_dir = comm_fuzz_run_dir();
  if (!ensure_dir_recursive(run_dir))
  {
    ROS_WARN("[comm_full_dump] failed to create report dir: %s", run_dir.c_str());
    return;
  }

  const string path = run_dir + "/" + comm_full_dump_file_name_;
  std::ofstream ofs(path.c_str(), std::ios::out | std::ios::trunc);
  if (!ofs.is_open())
  {
    ROS_WARN("[comm_full_dump] failed to open dump file: %s", path.c_str());
    return;
  }

  comm_full_dump_triggered_ = true;
  comm_full_dump_trigger_reason_ = reason;
  comm_full_dump_trigger_ros_time_ = ros::Time::now().toSec();

  ofs << "{\"event\":\"dump_trigger\",\"reason\":\"" << json_escape(reason)
      << "\",\"ros_time\":" << comm_full_dump_trigger_ros_time_
      << ",\"wall_time\":\"" << json_escape(now_iso8601()) << "\"}\n";
  for (std::deque<string>::const_iterator it = comm_full_dump_buffer_.begin(); it != comm_full_dump_buffer_.end(); ++it)
  {
    ofs << *it << "\n";
    comm_full_dump_written_count_++;
  }
  comm_full_dump_buffer_.clear();
  ROS_WARN("[comm_full_dump] triggered by %s, dump file: %s", reason.c_str(), path.c_str());
}

void comm_incident_cb(const std_msgs::StringConstPtr &msg)
{
  comm_full_dump_trigger("collision_event");
}

void comm_mandatory_stop_cb(const std_msgs::EmptyConstPtr &msg)
{
  (void)msg;
  comm_full_dump_trigger("mandatory_stop");
}

unsigned long long comm_fuzz_next_trace_id()
{
  boost::lock_guard<boost::mutex> lock(comm_fuzz_trace_mtx_);
  return ++comm_fuzz_trace_seq_;
}

void comm_fuzz_append_trace(const string &event,
                            const traj_utils::MINCOTraj &msg,
                            const bool window_hit,
                            const double planned_delay_ms,
                            const double queue_wait_ms,
                            const string &note,
                            const unsigned long long trace_id)
{
  if (!comm_fuzz_enable_ || !comm_fuzz_trace_enable_)
    return;

  const string run_dir = comm_fuzz_run_dir();
  if (!ensure_dir_recursive(run_dir))
    return;

  const string path = run_dir + "/comm_fuzz_trace.jsonl";
  std::ofstream ofs(path.c_str(), std::ios::out | std::ios::app);
  if (!ofs.is_open())
    return;

  const double now_ros = ros::Time::now().toSec();
  const double now_wall = ros::WallTime::now().toSec();
  const double msg_age_ms = (now_ros - msg.start_time.toSec()) * 1000.0;

  ofs << std::fixed << std::setprecision(6);
  ofs << "{";
  ofs << "\"trace_id\":" << trace_id << ",";
  ofs << "\"event\":\"" << json_escape(event) << "\",";
  ofs << "\"wall_time\":\"" << json_escape(now_iso8601()) << "\",";
  ofs << "\"ros_time\":" << now_ros << ",";
  ofs << "\"wall_time_sec\":" << now_wall << ",";
  ofs << "\"drone_id\":" << msg.drone_id << ",";
  ofs << "\"traj_id\":" << msg.traj_id << ",";
  ofs << "\"msg_start_time\":" << msg.start_time.toSec() << ",";
  ofs << "\"msg_age_ms\":" << msg_age_ms << ",";
  ofs << "\"window_hit\":" << (window_hit ? "true" : "false") << ",";
  ofs << "\"mode\":\"" << json_escape(comm_fuzz_mode_used_) << "\",";
  ofs << "\"seed\":" << comm_fuzz_seed_used_ << ",";
  ofs << "\"target_drone_id\":" << comm_fuzz_target_drone_id_ << ",";
  ofs << "\"window_start_s\":" << comm_fuzz_window_start_s_ << ",";
  ofs << "\"window_end_s\":" << comm_fuzz_window_end_s_ << ",";
  ofs << "\"factor_name\":\"" << json_escape(comm_fuzz_factor_name_) << "\",";
  ofs << "\"factor_value\":" << comm_fuzz_factor_value_ << ",";
  ofs << "\"in_topic\":\"/broadcast_traj_from_planner\",";
  ofs << "\"out_topic\":\"udp_broadcast\",";
  ofs << "\"planned_delay_ms\":" << planned_delay_ms << ",";
  ofs << "\"queue_wait_ms\":" << queue_wait_ms << ",";
  ofs << "\"note\":\"" << json_escape(note) << "\"";
  ofs << "}" << "\n";
}

enum MESSAGE_TYPE
{
  ODOM = 100,
  ONE_TRAJ,
  STOP,
  GOAL,
  JOY
} massage_type_;

int init_broadcast(const char *ip, const int port)
{
  int fd;

  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
  {
    ROS_ERROR("[bridge_node]Socket sender creation error!");
    exit(EXIT_FAILURE);
  }

  int so_broadcast = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0)
  {
    cout << "Error in setting Broadcast option";
    exit(EXIT_FAILURE);
  }

  addr_udp_send_.sin_family = AF_INET;
  addr_udp_send_.sin_port = htons(port);

  if (inet_pton(AF_INET, ip, &addr_udp_send_.sin_addr) <= 0)
  {
    printf("\nInvalid address/ Address not supported \n");
    return -1;
  }

  return fd;
}

string now_iso8601()
{
  time_t raw_t = time(nullptr);
  struct tm tm_buf;
  localtime_r(&raw_t, &tm_buf);
  char out[64];
  strftime(out, sizeof(out), "%Y-%m-%dT%H:%M:%S", &tm_buf);
  return string(out);
}

bool ensure_dir_recursive(const string &path)
{
  if (path.empty())
    return false;

  string cur;
  if (path[0] == '/')
    cur = "/";

  string part;
  stringstream ss(path);
  while (getline(ss, part, '/'))
  {
    if (part.empty())
      continue;

    if (!cur.empty() && cur.back() != '/')
      cur += "/";
    cur += part;

    if (mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST)
      return false;
  }
  return true;
}

string json_escape(const string &s)
{
  string out;
  out.reserve(s.size() + 8);
  for (size_t i = 0; i < s.size(); ++i)
  {
    const char c = s[i];
    switch (c)
    {
    case '\\':
      out += "\\\\";
      break;
    case '"':
      out += "\\\"";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\r':
      out += "\\r";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      out += c;
      break;
    }
  }
  return out;
}

template <typename T>
void send_udp_message(const MESSAGE_TYPE msg_type, const T &msg, const char *err_tag)
{
  if (!comm_fuzz_enable_ && legacy_passthrough_when_no_fuzz_)
  {
    int len = serializeTopic(msg_type, msg);
    if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
    {
      ROS_ERROR("UDP SEND ERROR (%s)!!!", err_tag);
    }
    return;
  }

  boost::lock_guard<boost::mutex> send_guard(udp_send_mtx_);
  int len = serializeTopic(msg_type, msg);
  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    ROS_ERROR("UDP SEND ERROR (%s)!!!", err_tag);
  }
}

bool comm_fuzz_in_window()
{
  double t = ros::Time::now().toSec() - comm_fuzz_start_ros_time_;
  return t >= comm_fuzz_window_start_s_ && t <= comm_fuzz_window_end_s_;
}

traj_utils::MINCOTraj comm_fuzz_apply_noise(const traj_utils::MINCOTraj &in_msg)
{
  traj_utils::MINCOTraj out_msg = in_msg;
  const double sigma = std::max(comm_fuzz_factor_value_, 0.0);
  std::normal_distribution<double> noise_dist(0.0, sigma);

  for (size_t i = 0; i < out_msg.start_p.size(); ++i)
    out_msg.start_p[i] = static_cast<float>(out_msg.start_p[i] + noise_dist(comm_fuzz_rng_));
  for (size_t i = 0; i < out_msg.end_p.size(); ++i)
    out_msg.end_p[i] = static_cast<float>(out_msg.end_p[i] + noise_dist(comm_fuzz_rng_));
  for (size_t i = 0; i < out_msg.inner_x.size(); ++i)
    out_msg.inner_x[i] = static_cast<float>(out_msg.inner_x[i] + noise_dist(comm_fuzz_rng_));
  for (size_t i = 0; i < out_msg.inner_y.size(); ++i)
    out_msg.inner_y[i] = static_cast<float>(out_msg.inner_y[i] + noise_dist(comm_fuzz_rng_));
  for (size_t i = 0; i < out_msg.inner_z.size(); ++i)
    out_msg.inner_z[i] = static_cast<float>(out_msg.inner_z[i] + noise_dist(comm_fuzz_rng_));

  return out_msg;
}

void comm_fuzz_write_meta(const bool final)
{
  if (!comm_fuzz_enable_)
    return;

  string run_id = comm_fuzz_run_id_;
  if (run_id.empty())
    run_id = "bridge_fuzz";

  const string run_dir = comm_fuzz_run_dir();
  if (!ensure_dir_recursive(run_dir))
  {
    ROS_WARN("[comm_fuzz_bridge] failed to create report dir: %s", run_dir.c_str());
    return;
  }

  const string path = run_dir + "/comm_fuzz_meta.json";
  std::ofstream ofs(path.c_str(), std::ios::out | std::ios::trunc);
  if (!ofs.is_open())
  {
    ROS_WARN("[comm_fuzz_bridge] failed to open meta file: %s", path.c_str());
    return;
  }

  boost::lock_guard<boost::mutex> lock(comm_fuzz_meta_mtx_);
  ofs << "{\n";
  ofs << "  \"run_id\": \"" << json_escape(run_id) << "\",\n";
  ofs << "  \"start_wall_time\": \"" << json_escape(comm_fuzz_start_wall_time_) << "\",\n";
  ofs << "  \"start_ros_time\": " << comm_fuzz_start_ros_time_ << ",\n";
  ofs << "  \"enabled\": " << (comm_fuzz_enable_ ? "true" : "false") << ",\n";
  ofs << "  \"seed\": " << comm_fuzz_seed_used_ << ",\n";
  ofs << "  \"target_drone_id\": " << comm_fuzz_target_drone_id_ << ",\n";
  ofs << "  \"mode\": \"" << json_escape(comm_fuzz_mode_used_) << "\",\n";
  ofs << "  \"window_start_s\": " << comm_fuzz_window_start_s_ << ",\n";
  ofs << "  \"window_duration_s\": " << comm_fuzz_window_duration_s_ << ",\n";
  ofs << "  \"window_end_s\": " << comm_fuzz_window_end_s_ << ",\n";
  ofs << "  \"factor_name\": \"" << json_escape(comm_fuzz_factor_name_) << "\",\n";
  ofs << "  \"factor_value\": " << comm_fuzz_factor_value_ << ",\n";
  ofs << "  \"in_topic\": \"/broadcast_traj_from_planner\",\n";
  ofs << "  \"out_topic\": \"udp_broadcast\",\n";
  ofs << "  \"msg_seen\": " << comm_fuzz_msg_seen_ << ",\n";
  ofs << "  \"msg_fuzzed\": " << comm_fuzz_msg_fuzzed_ << ",\n";
  ofs << "  \"trace_enabled\": " << (comm_fuzz_trace_enable_ ? "true" : "false") << ",\n";
  ofs << "  \"trace_file\": \"" << json_escape(run_dir + "/comm_fuzz_trace.jsonl") << "\",\n";
  ofs << "  \"full_dump_enabled\": " << (comm_full_dump_enable_ ? "true" : "false") << ",\n";
  ofs << "  \"full_dump_triggered\": " << (comm_full_dump_triggered_ ? "true" : "false") << ",\n";
  ofs << "  \"full_dump_trigger_reason\": \"" << json_escape(comm_full_dump_trigger_reason_) << "\",\n";
  ofs << "  \"full_dump_trigger_ros_time\": " << comm_full_dump_trigger_ros_time_ << ",\n";
  ofs << "  \"full_dump_file\": \"" << json_escape(run_dir + "/" + comm_full_dump_file_name_) << "\",\n";
  ofs << "  \"full_dump_buffered_count\": " << comm_full_dump_buffered_count_ << ",\n";
  ofs << "  \"full_dump_written_count\": " << comm_full_dump_written_count_ << ",\n";
  ofs << "  \"full_dump_drop_count\": " << comm_full_dump_drop_count_ << ",\n";
  ofs << "  \"final\": " << (final ? "true" : "false") << ",\n";
  ofs << "  \"wall_time\": \"" << json_escape(now_iso8601()) << "\"\n";
  ofs << "}\n";
}

void comm_fuzz_delay_worker()
{
  while (true)
  {
    ScheduledTraj item;
    {
      boost::unique_lock<boost::mutex> lock(comm_fuzz_delay_mtx_);
      while (comm_fuzz_delay_worker_running_ && comm_fuzz_delay_queue_.empty())
      {
        comm_fuzz_delay_cv_.wait(lock);
      }

      if (!comm_fuzz_delay_worker_running_ && comm_fuzz_delay_queue_.empty())
        return;

      if (comm_fuzz_delay_queue_.empty())
        continue;

      ros::WallTime now = ros::WallTime::now();
      if (comm_fuzz_delay_queue_.front().due > now)
      {
        const ros::WallDuration wait_dur = comm_fuzz_delay_queue_.front().due - now;
        const long wait_ms = std::max(1L, static_cast<long>(wait_dur.toSec() * 1000.0));
        const boost::system_time wait_until = boost::get_system_time() + boost::posix_time::milliseconds(wait_ms);
        comm_fuzz_delay_cv_.timed_wait(lock, wait_until);
        continue;
      }

      item = comm_fuzz_delay_queue_.front();
      comm_fuzz_delay_queue_.pop_front();
    }

    const double queue_wait_ms = (ros::WallTime::now() - item.enqueued).toSec() * 1000.0;
    send_udp_message(MESSAGE_TYPE::ONE_TRAJ, item.msg, "2-delay");
    comm_full_dump_append(item.msg, "tx", item.trace_event, "sent_from_delay_queue");
    comm_fuzz_append_trace(item.trace_event,
                           item.msg,
                           true,
                           item.planned_delay_s * 1000.0,
                           std::max(0.0, queue_wait_ms),
                           "sent_from_delay_queue",
                           item.trace_id);
  }
}

void enqueue_delayed_traj(const traj_utils::MINCOTraj &msg,
                         double delay_s,
                         const unsigned long long trace_id,
                         const string &trace_event)
{
  ScheduledTraj item;
  item.msg = msg;
  item.enqueued = ros::WallTime::now();
  item.due = ros::WallTime::now() + ros::WallDuration(std::max(delay_s, 0.0));
  item.planned_delay_s = std::max(delay_s, 0.0);
  item.trace_id = trace_id;
  item.trace_event = trace_event;

  boost::lock_guard<boost::mutex> lock(comm_fuzz_delay_mtx_);
  std::deque<ScheduledTraj>::iterator it = comm_fuzz_delay_queue_.begin();
  for (; it != comm_fuzz_delay_queue_.end(); ++it)
  {
    if (item.due < it->due)
      break;
  }
  comm_fuzz_delay_queue_.insert(it, item);
  comm_fuzz_delay_cv_.notify_one();
}

int udp_bind_to_port(const int port, int &server_fd)
{
  struct sockaddr_in address;
  int opt = 1;

  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
  {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }

  // Forcefully attaching socket to the port
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                 &opt, sizeof(opt)))
  {
    perror("setsockopt");
    exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port);

  // Forcefully attaching socket to the port
  if (bind(server_fd, (struct sockaddr *)&address,
           sizeof(address)) < 0)
  {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  return server_fd;
}

template <typename T>
int serializeTopic(const MESSAGE_TYPE msg_type, const T &msg)
{
  auto ptr = (uint8_t *)(udp_send_buf_);

  *((MESSAGE_TYPE*)ptr) = msg_type;
  ptr += sizeof(MESSAGE_TYPE);

  namespace ser = ros::serialization;
  uint32_t msg_size = ser::serializationLength(msg);

  *((uint32_t *)ptr) = msg_size;
  ptr += sizeof(uint32_t);

  ser::OStream stream(ptr, msg_size);
  ser::serialize(stream, msg);

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

template <typename T>
int deserializeTopic(T &msg)
{
  auto ptr = (uint8_t *)(udp_recv_buf_ + sizeof(MESSAGE_TYPE));

  uint32_t msg_size = *((uint32_t *)ptr);
  ptr += sizeof(uint32_t);

  namespace ser = ros::serialization;
  ser::IStream stream(ptr, msg_size);
  ser::deserialize(stream, msg);

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

void odom_sub_udp_cb(const nav_msgs::OdometryPtr &msg)
{

  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() * odom_broadcast_freq_ < 1.0)
  {
    return;
  }
  t_last = t_now;

  msg->child_frame_id = string("drone_") + std::to_string(drone_id_);

  send_udp_message(MESSAGE_TYPE::ODOM, *msg, "1");
}

void one_traj_sub_udp_cb(const traj_utils::MINCOTrajPtr &msg)
{
  if (!comm_fuzz_enable_ && legacy_passthrough_when_no_fuzz_)
  {
    comm_full_dump_append(*msg, "tx", "send_passthrough_no_fuzz", "legacy_passthrough_when_no_fuzz");
    send_udp_message(MESSAGE_TYPE::ONE_TRAJ, *msg, "2");
    return;
  }

  {
    boost::lock_guard<boost::mutex> lock(comm_fuzz_meta_mtx_);
    comm_fuzz_msg_seen_++;
  }

  const bool is_target = msg->drone_id == comm_fuzz_target_drone_id_;
  const bool in_window = comm_fuzz_in_window();

  if (!comm_fuzz_enable_ || !is_target || !in_window)
  {
    if (comm_fuzz_enable_ && is_target)
    {
      const unsigned long long trace_id = comm_fuzz_next_trace_id();
      const string note = in_window ? "pass_through_target" : "pass_through_window_miss";
      comm_fuzz_append_trace("send_passthrough", *msg, in_window, 0.0, 0.0, note, trace_id);
    }
    comm_full_dump_append(*msg, "tx", "send_passthrough", in_window ? "target_in_window_passthrough" : "window_miss_passthrough");
    send_udp_message(MESSAGE_TYPE::ONE_TRAJ, *msg, "2");
    return;
  }

  {
    boost::lock_guard<boost::mutex> lock(comm_fuzz_meta_mtx_);
    comm_fuzz_msg_fuzzed_++;
  }

  if (comm_fuzz_mode_used_ == "delay")
  {
    const double delay_s = std::max(comm_fuzz_factor_value_, 0.0) / 1000.0;
    const unsigned long long trace_id = comm_fuzz_next_trace_id();
    comm_fuzz_append_trace("enqueue_delay", *msg, true, delay_s * 1000.0, 0.0, "scheduled", trace_id);
    comm_full_dump_append(*msg, "tx", "enqueue_delay", "scheduled");
    enqueue_delayed_traj(*msg, delay_s, trace_id, "send_delay");
    return;
  }

  if (comm_fuzz_mode_used_ == "jitter")
  {
    std::uniform_real_distribution<double> dist(0.0, std::max(comm_fuzz_factor_value_, 0.0) / 1000.0);
    const double delay_s = dist(comm_fuzz_rng_);
    const unsigned long long trace_id = comm_fuzz_next_trace_id();
    comm_fuzz_append_trace("enqueue_jitter", *msg, true, delay_s * 1000.0, 0.0, "scheduled", trace_id);
    comm_full_dump_append(*msg, "tx", "enqueue_jitter", "scheduled");
    enqueue_delayed_traj(*msg, delay_s, trace_id, "send_jitter");
    return;
  }

  const traj_utils::MINCOTraj noisy = comm_fuzz_apply_noise(*msg);
  const unsigned long long trace_id = comm_fuzz_next_trace_id();
  comm_fuzz_append_trace("send_noise", noisy, true, 0.0, 0.0, "sent_immediately", trace_id);
  comm_full_dump_append(noisy, "tx", "send_noise", "sent_immediately");
  send_udp_message(MESSAGE_TYPE::ONE_TRAJ, noisy, "2-noise");
}

void mandatory_stop_sub_udp_cb(const std_msgs::EmptyPtr &msg)
{
  send_udp_message(MESSAGE_TYPE::STOP, *msg, "3");
}

void goal_sub_udp_cb(const quadrotor_msgs::GoalSetPtr &msg)
{
  send_udp_message(MESSAGE_TYPE::GOAL, *msg, "4");
}

void joy_sub_udp_cb(const sensor_msgs::JoyPtr &msg)
{
  send_udp_message(MESSAGE_TYPE::JOY, *msg, "5");
}

void udp_recv_fun()
{
  int valread;
  struct sockaddr_in addr_client;
  socklen_t addr_len = sizeof(addr_client);

  // Connect
  if (udp_bind_to_port(UDP_PORT, udp_server_fd_) < 0)
  {
    ROS_ERROR("[bridge_node]Socket recever creation error!");
    exit(EXIT_FAILURE);
  }

  while (udp_recv_running_ && ros::ok())
  {
    addr_len = sizeof(addr_client);
    if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUF_LEN, 0, (struct sockaddr *)&addr_client, (socklen_t *)&addr_len)) < 0)
    {
      if (!udp_recv_running_ || !ros::ok())
      {
        break;
      }
      if (errno == EINTR)
      {
        continue;
      }
      perror("recvfrom() < 0, error:");
      break;
    }

    char *ptr = udp_recv_buf_;
    switch (*((MESSAGE_TYPE *)ptr))
    {

    case MESSAGE_TYPE::ODOM:
    {
      if (valread == deserializeTopic(odom_msg_))
      {
        other_odoms_pub_.publish(odom_msg_);
      }
      else
      {
        ROS_ERROR("Received message length not matches the sent one (2)!!!");
        continue;
      }

      break;
    }

    case MESSAGE_TYPE::ONE_TRAJ:
    {

      if (valread == deserializeTopic(MINCOTraj_msg_))
      {
        comm_full_dump_append(MINCOTraj_msg_, "rx", "recv_udp", "broadcast_traj_to_planner");
        one_traj_pub_.publish(MINCOTraj_msg_);
      }
      else
      {
        ROS_ERROR("Received message length not matches the sent one (2)!!!");
        continue;
      }

      break;
    }

    case MESSAGE_TYPE::STOP:
    {

      if (valread == deserializeTopic(stop_msg_))
      {
        comm_full_dump_trigger("udp_stop_message");
        mandatory_stop_pub_.publish(stop_msg_);
      }
      else
      {
        ROS_ERROR("Received message length not matches the sent one (3)!!!");
        continue;
      }

      break;
    }

    case MESSAGE_TYPE::GOAL:
    {
      
      if (valread == deserializeTopic(goal_msg_))
      {
        goal_pub_.publish(goal_msg_);
      }
      else
      {
        ROS_ERROR("Received message length not matches the sent one (4)!!!");
        continue;
      }

      break;
    }

    case MESSAGE_TYPE::JOY:
    {
      
      if (valread == deserializeTopic(joy_msg_))
      {
        joy_pub_.publish(joy_msg_);
      }
      else
      {
        ROS_ERROR("Received message length not matches the sent one (5)!!!");
        continue;
      }

      break;
    }

    default:

      ROS_ERROR("Unknown received message type???");

      break;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  nh.param("broadcast_ip", udp_ip_, string("127.0.0.255"));
  nh.param("drone_id", drone_id_, -1);
  nh.param("odom_max_freq", odom_broadcast_freq_, 1000.0);

  nh.param("enable_comm_fuzz", comm_fuzz_enable_, false);
  nh.param("legacy_passthrough_when_no_fuzz", legacy_passthrough_when_no_fuzz_, true);
  nh.param("comm_fuzz_target_drone_id", comm_fuzz_target_drone_id_, 0);
  nh.param("comm_fuzz_mode", comm_fuzz_mode_cfg_, string("random"));
  nh.param("comm_fuzz_seed", comm_fuzz_seed_cfg_, -1);
  nh.param("comm_fuzz_start_min_sec", comm_fuzz_start_min_s_, 5.0);
  nh.param("comm_fuzz_start_max_sec", comm_fuzz_start_max_s_, 25.0);
  nh.param("comm_fuzz_duration_min_sec", comm_fuzz_duration_min_s_, 5.0);
  nh.param("comm_fuzz_duration_max_sec", comm_fuzz_duration_max_s_, 20.0);
  nh.param("comm_fuzz_delay_min_ms", comm_fuzz_delay_min_ms_, 80.0);
  nh.param("comm_fuzz_delay_max_ms", comm_fuzz_delay_max_ms_, 250.0);
  nh.param("comm_fuzz_jitter_min_ms", comm_fuzz_jitter_min_ms_, 30.0);
  nh.param("comm_fuzz_jitter_max_ms", comm_fuzz_jitter_max_ms_, 120.0);
  nh.param("comm_fuzz_noise_sigma_pos_min", comm_fuzz_noise_sigma_pos_min_, 0.05);
  nh.param("comm_fuzz_noise_sigma_pos_max", comm_fuzz_noise_sigma_pos_max_, 0.20);
  nh.param("comm_fuzz_report_dir", comm_fuzz_report_dir_, string("~/.ego_fuzz_reports"));
  nh.param("comm_fuzz_run_id", comm_fuzz_run_id_, string(""));
  nh.param("enable_comm_fuzz_trace", comm_fuzz_trace_enable_, true);
  nh.param("enable_comm_full_dump_on_incident", comm_full_dump_enable_, true);
  nh.param("comm_full_dump_buffer_max", comm_full_dump_buffer_max_, 4000);
  nh.param("comm_full_dump_file_name", comm_full_dump_file_name_, string("comm_full_dump_on_incident.jsonl"));

  comm_fuzz_msg_seen_ = 0;
  comm_fuzz_msg_fuzzed_ = 0;
  comm_fuzz_trace_seq_ = 0;
  comm_fuzz_start_wall_time_ = now_iso8601();
  comm_fuzz_start_ros_time_ = ros::Time::now().toSec();
  comm_fuzz_delay_worker_running_ = false;
  comm_full_dump_triggered_ = false;
  comm_full_dump_trigger_reason_ = "";
  comm_full_dump_trigger_ros_time_ = -1.0;
  comm_full_dump_buffered_count_ = 0;
  comm_full_dump_written_count_ = 0;
  comm_full_dump_drop_count_ = 0;

  // Keep no-fuzz path close to historical bridge behavior.
  if (comm_fuzz_enable_)
  {
    comm_fuzz_delay_worker_running_ = true;

    if (comm_fuzz_start_min_s_ > comm_fuzz_start_max_s_)
      std::swap(comm_fuzz_start_min_s_, comm_fuzz_start_max_s_);
    if (comm_fuzz_duration_min_s_ > comm_fuzz_duration_max_s_)
      std::swap(comm_fuzz_duration_min_s_, comm_fuzz_duration_max_s_);
    if (comm_fuzz_delay_min_ms_ > comm_fuzz_delay_max_ms_)
      std::swap(comm_fuzz_delay_min_ms_, comm_fuzz_delay_max_ms_);
    if (comm_fuzz_jitter_min_ms_ > comm_fuzz_jitter_max_ms_)
      std::swap(comm_fuzz_jitter_min_ms_, comm_fuzz_jitter_max_ms_);
    if (comm_fuzz_noise_sigma_pos_min_ > comm_fuzz_noise_sigma_pos_max_)
      std::swap(comm_fuzz_noise_sigma_pos_min_, comm_fuzz_noise_sigma_pos_max_);

    comm_fuzz_seed_used_ = comm_fuzz_seed_cfg_ >= 0 ? comm_fuzz_seed_cfg_ : static_cast<int>(time(nullptr) & 0x7fffffff);
    comm_fuzz_rng_.seed(comm_fuzz_seed_used_);

    std::uniform_real_distribution<double> start_dist(comm_fuzz_start_min_s_, comm_fuzz_start_max_s_);
    std::uniform_real_distribution<double> duration_dist(comm_fuzz_duration_min_s_, comm_fuzz_duration_max_s_);
    comm_fuzz_window_start_s_ = start_dist(comm_fuzz_rng_);
    comm_fuzz_window_duration_s_ = duration_dist(comm_fuzz_rng_);
    comm_fuzz_window_end_s_ = comm_fuzz_window_start_s_ + comm_fuzz_window_duration_s_;

    if (comm_fuzz_mode_cfg_ == "random")
    {
      static const string modes[3] = {"delay", "jitter", "noise"};
      std::uniform_int_distribution<int> idx_dist(0, 2);
      comm_fuzz_mode_used_ = modes[idx_dist(comm_fuzz_rng_)];
    }
    else if (comm_fuzz_mode_cfg_ == "delay" || comm_fuzz_mode_cfg_ == "jitter" || comm_fuzz_mode_cfg_ == "noise")
    {
      comm_fuzz_mode_used_ = comm_fuzz_mode_cfg_;
    }
    else
    {
      comm_fuzz_mode_used_ = "delay";
    }

    if (comm_fuzz_mode_used_ == "delay")
    {
      std::uniform_real_distribution<double> d(comm_fuzz_delay_min_ms_, comm_fuzz_delay_max_ms_);
      comm_fuzz_factor_name_ = "delay_ms";
      comm_fuzz_factor_value_ = d(comm_fuzz_rng_);
    }
    else if (comm_fuzz_mode_used_ == "jitter")
    {
      std::uniform_real_distribution<double> d(comm_fuzz_jitter_min_ms_, comm_fuzz_jitter_max_ms_);
      comm_fuzz_factor_name_ = "jitter_ms";
      comm_fuzz_factor_value_ = d(comm_fuzz_rng_);
    }
    else
    {
      std::uniform_real_distribution<double> d(comm_fuzz_noise_sigma_pos_min_, comm_fuzz_noise_sigma_pos_max_);
      comm_fuzz_factor_name_ = "noise_sigma_pos";
      comm_fuzz_factor_value_ = d(comm_fuzz_rng_);
    }
  }

  if (comm_fuzz_run_id_.empty())
    comm_fuzz_run_id_ = "bridge_" + std::to_string(static_cast<long>(time(nullptr)));

  if (comm_fuzz_enable_)
  {
    ROS_INFO("[comm_fuzz_bridge] enable=true mode=%s target=%d seed=%d window=[%.3f, %.3f] factor=%s:%.6f",
             comm_fuzz_mode_used_.c_str(), comm_fuzz_target_drone_id_, comm_fuzz_seed_used_,
             comm_fuzz_window_start_s_, comm_fuzz_window_end_s_, comm_fuzz_factor_name_.c_str(), comm_fuzz_factor_value_);
    comm_fuzz_write_meta(false);
  }

  if (drone_id_ == -1)
  {
    ROS_WARN("[swarm bridge] Wrong drone_id!");
    exit(EXIT_FAILURE);
  }

  other_odoms_sub_ = nh.subscribe("my_odom", 10, odom_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  other_odoms_pub_ = nh.advertise<nav_msgs::Odometry>("/others_odom", 10);

  one_traj_sub_ = nh.subscribe("/broadcast_traj_from_planner", 100, one_traj_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  one_traj_pub_ = nh.advertise<traj_utils::MINCOTraj>("/broadcast_traj_to_planner", 100);
  comm_incident_sub_ = nh.subscribe("/fuzz_collision_event", 20, comm_incident_cb, ros::TransportHints().tcpNoDelay());
  comm_mandatory_stop_sub_ = nh.subscribe("/mandatory_stop_to_planner", 20, comm_mandatory_stop_cb, ros::TransportHints().tcpNoDelay());

  // mandatory_stop_sub_ = nh.subscribe("/mandatory_stop_from_users", 100, mandatory_stop_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  // mandatory_stop_pub_ = nh.advertise<std_msgs::Empty>("/mandatory_stop_to_planner", 100);

  goal_sub_ = nh.subscribe("/goal_user2brig", 100, goal_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal_brig2plner", 100);

  joy_sub_ = nh.subscribe("/joystick_from_users", 100, joy_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  joy_pub_ = nh.advertise<sensor_msgs::Joy>("/joystick_from_bridge", 100);

  udp_recv_running_ = true;
  boost::thread udp_recv_thd(udp_recv_fun);

  boost::thread *comm_fuzz_delay_thd = nullptr;
  if (comm_fuzz_enable_)
  {
    comm_fuzz_delay_thd = new boost::thread(comm_fuzz_delay_worker);
  }
  ros::Duration(0.1).sleep();

  // UDP connect
  udp_send_fd_ = init_broadcast(udp_ip_.c_str(), UDP_PORT);

  cout << "[rosmsg_tcp_bridge] start running" << endl;

  ros::spin();

  if (comm_fuzz_enable_)
  {
    comm_fuzz_write_meta(true);
  }

  udp_recv_running_ = false;
  if (udp_server_fd_ > 0)
  {
    close(udp_server_fd_);
    udp_server_fd_ = -1;
  }
  udp_recv_thd.join();

  if (comm_fuzz_delay_thd != nullptr)
  {
    {
      boost::lock_guard<boost::mutex> lock(comm_fuzz_delay_mtx_);
      comm_fuzz_delay_worker_running_ = false;
      comm_fuzz_delay_cv_.notify_all();
    }
    comm_fuzz_delay_thd->join();
    delete comm_fuzz_delay_thd;
  }

  if (udp_send_fd_ > 0)
  {
    close(udp_send_fd_);
    udp_send_fd_ = -1;
  }

  return 0;
}
