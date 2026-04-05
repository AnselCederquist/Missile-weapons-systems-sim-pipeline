import pandas as pd

df = pd.read_csv('D:/Weapons-systems-sim-pipeline/07_thermal_structural/results/aeroheating.csv')

def write_xml(times, values, filename, quantity="Heat Flux", unit="W m^-2"):
    lines = ['<?xml version="1.0" encoding="utf-8"?>']
    lines.append('<DataBlock>')
    lines.append(f'  <Header><Quantity>{quantity}</Quantity><Unit>{unit}</Unit></Header>')
    lines.append('  <Data>')
    for t, v in zip(times, values):
        lines.append(f'    <Row><Time unit="s">{t:.4f}</Time><Value>{v:.4f}</Value></Row>')
    lines.append('  </Data>')
    lines.append('</DataBlock>')
    with open(filename, 'w') as f:
        f.write('\n'.join(lines))
    print(f"Written: {filename} ({len(times)} rows)")

write_xml(df['time_s'], df['q_fin_LE_Wm2'],
          'D:/Weapons-systems-sim-pipeline/07_thermal_structural/results/fin_LE_heatflux.xml')

write_xml(df['time_s'], df['q_nose_Wm2'],
          'D:/Weapons-systems-sim-pipeline/07_thermal_structural/results/nose_heatflux.xml')