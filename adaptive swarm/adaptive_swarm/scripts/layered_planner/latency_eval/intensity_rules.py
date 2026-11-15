"""Strict import of the supplied workbook; conditions retain cell provenance."""
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from zipfile import ZipFile

SHEETS={'机间低净空':'drone_drone','机障低净空':'drone_obstacle','死锁代理':'deadlock',
        '完成时间偏长':'delay','规划代理':'planning','广播读取偏移':'drift'}
FIELDS={
 '四机最大当前目标点偏差':('cluster_max_target_offset','m'),
 '源机当前目标点偏差':('source_target_offset','m'),
 '任务已用时间':('elapsed_task_time','s'),
 '领机到任务终点距离':('leader_goal_distance','m'),
 '全局设定点进度':('setpoint_progress',''),
 '注入前最小机间距':('current_min_pair_distance','m'),
 '注入前最小机障距离':('current_min_obstacle_distance','m'),
 '四机最大位移方向夹角':('cluster_max_course_angle','°'),
 '源机相邻位移转角':('source_turn_angle','°'),
 '源机位置差分加速度模长':('source_acceleration','m/s²'),
 '四机平均位置差分加速度模长':('cluster_mean_acceleration','m/s²'),
}


def workbook_cells(path):
    ns={'s':'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}
    with ZipFile(path) as archive:
        shared=[]
        if 'xl/sharedStrings.xml' in archive.namelist():
            shared=[''.join(item.itertext()) for item in ET.fromstring(archive.read('xl/sharedStrings.xml'))]
        links={r.attrib['Id']:r.attrib['Target'] for r in ET.fromstring(archive.read('xl/_rels/workbook.xml.rels'))}
        sheets=ET.fromstring(archive.read('xl/workbook.xml')).find('s:sheets',ns)
        result={}
        for sheet in sheets:
            target=links[sheet.attrib['{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id']]
            member=target.lstrip('/') if target.startswith('/') else 'xl/'+target
            cells={}
            for cell in ET.fromstring(archive.read(member)).findall('.//s:sheetData/s:row/s:c',ns):
                if (cell.find('s:f',ns) is not None and sheet.attrib['name'] in SHEETS
                        and cell.attrib['r'].startswith('B') and int(cell.attrib['r'][1:])>=4):
                    raise ValueError('formula rule cells require an explicit evaluated-rule review')
                value=cell.find('s:v',ns);text=value.text if value is not None else ''
                kind=cell.attrib.get('t')
                if kind=='s':text=shared[int(text)]
                elif kind=='inlineStr':text=''.join(cell.find('s:is',ns).itertext())
                if text:cells[cell.attrib['r']]=text
            result[sheet.attrib['name']]=cells
        return result


def parse_conditions(expression):
    atoms=[]
    for text in expression.split(' 且 '):
        match=re.fullmatch(r'(.+?)\s*(<=|>)\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)\s*(.*)',text.strip())
        if match is None:raise ValueError('unparsed rule condition: '+text)
        label,op,value,unit=match.groups()
        if label.strip() not in FIELDS:raise ValueError('unsupported rule variable: '+label)
        field,expected=FIELDS[label.strip()]
        if unit.strip()!=expected:raise ValueError('unit mismatch for '+label+': '+unit)
        number=float(value)
        if not math.isfinite(number):raise ValueError('nonfinite threshold')
        atoms.append(dict(field=field,op=op,threshold=math.radians(number) if expected=='°' else number,
                          unit='rad' if expected=='°' else expected,source_text=text))
    return atoms


def import_rules(path):
    path=Path(path);sheets=workbook_cells(path);groups=[];mapping=[]
    for name,target in SHEETS.items():
        if name not in sheets:raise ValueError('missing target sheet: '+name)
        cells=sheets[name];rules=[]
        for cell in sorted((c for c in cells if c.startswith('B') and c[1:].isdigit()),key=lambda c:int(c[1:])):
            if int(cell[1:])<4:continue
            expression=cells[cell]
            if ' 且 ' not in expression:raise ValueError('unrecognized rule row '+name+'!'+cell)
            identifier=target+'_xlsx_row_'+cell[1:];conditions=parse_conditions(expression)
            rule=dict(rule_id=identifier,name=cells.get('A'+cell[1:],''),expression=expression,conditions=conditions,source_sheet=name,source_cell=cell)
            rules.append(rule)
            for c in conditions:mapping.append(dict(target=target,rule_id=identifier,rule_name=rule['name'],expression=expression,
                field=c['field'],op=c['op'],threshold=c['threshold'],unit=c['unit'],source_sheet=name,source_cell=cell,
                acquisition='core.feature_vector before publish',limitation='current target offset is not trajectory deviation; displacement angle is not body yaw'))
        if not rules:raise ValueError('no rules in '+name)
        groups.append(dict(target=target,name=name,logic='OR',rules=rules))
    return dict(version=1,source_workbook=str(path.resolve()),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                groups=groups,mapping=mapping,source_cells=sheets,
                caveats=['Workbook outcome/baseline cells are not silently used as calibration.',
                         'Rows are imported exactly as present; title counts are not trusted.'])
