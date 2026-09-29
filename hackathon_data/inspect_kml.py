import xml.etree.ElementTree as ET
import glob
import os
import sys

data_dir = 'd:/New folder (3)/task_v1/hackathon_data'
for kml_file in glob.glob(os.path.join(data_dir, '*.kml')):
    fname = os.path.basename(kml_file)
    size_mb = os.path.getsize(kml_file) / (1024*1024)
    sys.stdout.buffer.write(f"\n========================================\n=== File: {fname} ({size_mb:.2f} MB) ===\n".encode('utf-8'))
    
    try:
        tree = ET.parse(kml_file)
        root = tree.getroot()
        ns = ''
        if root.tag.startswith('{'):
            ns = root.tag.split('}')[0] + '}'
        
        placemarks = root.findall(f'.//{ns}Placemark')
        sys.stdout.buffer.write(f"Placemarks count: {len(placemarks)}\n".encode('utf-8'))
        
        geo_types = set()
        folders = set()
        for f in root.findall(f'.//{ns}Folder'):
            fn = f.find(f'{ns}name')
            if fn is not None and fn.text:
                folders.add(fn.text)
                
        if folders:
            sample_folders = list(folders)[:8]
            sys.stdout.buffer.write(f"Folders count: {len(folders)}, sample: {sample_folders}\n".encode('utf-8'))
            
        sample_info = []
        for i, pm in enumerate(placemarks):
            name_node = pm.find(f'{ns}name')
            pname = name_node.text if name_node is not None else 'NoName'
            for child in pm:
                tag = child.tag.replace(ns, '')
                if tag in ['Polygon', 'Point', 'LineString', 'MultiGeometry', 'Model']:
                    geo_types.add(tag)
            if i < 4:
                desc_node = pm.find(f'{ns}description')
                desc = (desc_node.text[:80] + '...') if desc_node is not None and desc_node.text else ''
                ext_data = pm.find(f'{ns}ExtendedData')
                data_fields = []
                if ext_data is not None:
                    for d in ext_data.findall(f'.//{ns}Data'):
                        val = d.find(f'{ns}value')
                        data_fields.append((d.get('name'), val.text if val is not None else ''))
                    for sd in ext_data.findall(f'.//{ns}SimpleData'):
                        data_fields.append((sd.get('name'), sd.text if sd is not None else ''))
                sample_info.append((pname, desc, data_fields[:4]))
                
        sys.stdout.buffer.write(f"Geometry types found: {geo_types}\n".encode('utf-8'))
        for s in sample_info:
            sys.stdout.buffer.write(f"  Placemark: Name='{s[0]}', Desc='{s[1]}', ExtData={s[2]}\n".encode('utf-8'))
    except Exception as e:
        sys.stdout.buffer.write(f"Error parsing {fname}: {e}\n".encode('utf-8'))
