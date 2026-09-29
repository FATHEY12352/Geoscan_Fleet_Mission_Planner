import xml.etree.ElementTree as ET
import re
import sys

def parse_altitude_string(alt_str: str):
    """
    Parses Russian airspace altitude strings into structured bounds (meters).
    Examples:
      'От земли до 900 м (3000 фут) AMSL' -> (0.0, 900.0, True)
      'От 800 м (2700 фут) AMSL до FL90' -> (800.0, 2743.2, False)
      'От FL280 до FL400' -> (8534.4, 12192.0, False)
      'От земли до FL160' -> (0.0, 4876.8, True)
      'От земли до 450 м (1500 фут) AMSL' -> (0.0, 450.0, True)
    """
    lower_m = 0.0
    upper_m = 10000.0
    is_surface = False
    
    if not alt_str:
        return {'lower_m': 0.0, 'upper_m': 10000.0, 'is_surface': True, 'raw': ''}
    
    s = alt_str.lower().replace('\xa0', ' ')
    
    # Check if starts from ground
    if 'от земли' in s or 'gnd' in s or 'земли' in s:
        is_surface = True
        lower_m = 0.0
    
    # Check FL (Flight Level in hundreds of feet)
    fl_matches = re.findall(r'fl\s*(\d+)', s)
    m_matches = re.findall(r'(\d+)\s*м\b', s)
    
    # If "От XXX м"
    from_m = re.search(r'от\s+(\d+)\s*м', s)
    if from_m:
        lower_m = float(from_m.group(1))
        is_surface = False
    elif 'от fl' in s and fl_matches:
        lower_m = float(fl_matches[0]) * 30.48
        is_surface = False
        
    # Upper bound
    to_m = re.search(r'до\s+(\d+)\s*м', s)
    if to_m:
        upper_m = float(to_m.group(1))
    elif 'до fl' in s:
        # Find FL after 'до'
        fl_after = re.search(r'до\s+fl\s*(\d+)', s)
        if fl_after:
            upper_m = float(fl_after.group(1)) * 30.48
            
    return {
        'lower_m': round(lower_m, 1),
        'upper_m': round(upper_m, 1),
        'is_surface': is_surface,
        'raw': alt_str
    }

# Test on Московская зона.kml
kml_zone = 'd:/New folder (3)/task_v1/hackathon_data/Московская зона.kml'
tree = ET.parse(kml_zone)
root = tree.getroot()
ns = '{http://www.opengis.net/kml/2.2}'

parsed_records = []
for pm in root.findall(f'.//{ns}Placemark'):
    ext = pm.find(f'{ns}ExtendedData')
    d_dict = {}
    if ext is not None:
        for d in ext.findall(f'.//{ns}Data'):
            v = d.find(f'{ns}value')
            d_dict[d.get('name')] = v.text if v is not None else ''
    alt_parsed = parse_altitude_string(d_dict.get('Altitudes', ''))
    parsed_records.append((d_dict.get('Name', ''), d_dict.get('Type', ''), alt_parsed))

print(f"Total parsed NFZ: {len(parsed_records)}")
surface_nfz = [r for r in parsed_records if r[2]['is_surface']]
high_altitude_only = [r for r in parsed_records if not r[2]['is_surface'] and r[2]['lower_m'] > 150.0]

print(f"Zones starting from surface (GND NFZ affecting UAVs): {len(surface_nfz)}")
print(f"High altitude zones (Lower bound > 150m, safe for low-level UAV flight): {len(high_altitude_only)}")

print("\nSample high-altitude zones safely bypassed under 150m:")
for r in high_altitude_only[:5]:
    print(f"  Zone {r[0]} ({r[1]}): {r[2]['raw']} -> Lower Bound: {r[2]['lower_m']}m (Drone flies underneath!)")
