import xml.etree.ElementTree as ET
import sys

# 1. Inspect sample obstacles from obstacles_Московская область.kml
kml_file = 'd:/New folder (3)/task_v1/hackathon_data/obstacles_Московская область.kml'
tree = ET.parse(kml_file)
root = tree.getroot()
ns = '{http://www.opengis.net/kml/2.2}'

sys.stdout.buffer.write(b"=== Sample 3D Obstacles ===\n")
for i, pm in enumerate(root.findall(f'.//{ns}Placemark')[:5]):
    name = pm.find(f'{ns}name').text if pm.find(f'{ns}name') is not None else ''
    coords = pm.find(f'.//{ns}coordinates')
    coord_txt = coords.text.strip().replace('\n', ' ')[:100] if coords is not None else ''
    sys.stdout.buffer.write(f"Obstacle {i+1}: Name='{name}', Coords='{coord_txt}'\n".encode('utf-8'))

# 2. Inspect altitude strings and types from Московская зона.kml
kml_zone = 'd:/New folder (3)/task_v1/hackathon_data/Московская зона.kml'
tree_zone = ET.parse(kml_zone)
root_zone = tree_zone.getroot()

sys.stdout.buffer.write(b"\n=== Sample NFZ Altitudes & Types ===\n")
alt_types = set()
for pm in root_zone.findall(f'.//{ns}Placemark')[:20]:
    ext = pm.find(f'{ns}ExtendedData')
    d_dict = {}
    if ext is not None:
        for d in ext.findall(f'.//{ns}Data'):
            v = d.find(f'{ns}value')
            d_dict[d.get('name')] = v.text if v is not None else ''
    sys.stdout.buffer.write(f"NFZ: {d_dict}\n".encode('utf-8'))
