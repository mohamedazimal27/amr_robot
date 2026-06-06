#!/usr/bin/env python3
import os
import qrcode

def generate_station_model(station_id, output_dir):
    station_dir = os.path.join(output_dir, station_id)
    textures_dir = os.path.join(station_dir, 'materials', 'textures')
    os.makedirs(textures_dir, exist_ok=True)

    # 1. Generate QR Code image
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=10,
        border=4,
    )
    qr.add_data(station_id)
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white")
    
    img_path = os.path.join(textures_dir, f"{station_id}.png")
    img.save(img_path)
    print(f"Generated QR code texture: {img_path}")

    # 2. Write model.config
    config_content = f"""<?xml version="1.0"?>
<model>
  <name>{station_id}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>AMR Developer</name>
    <email>developer@amr.com</email>
  </author>
  <description>QR code station {station_id}</description>
</model>
"""
    config_path = os.path.join(station_dir, 'model.config')
    with open(config_path, 'w') as f:
        f.write(config_content)
    print(f"Generated model config: {config_path}")

    # 3. Write model.sdf
    # We define the origin at the bottom of the model (Z=0)
    # The support pillar is 0.1x0.1x0.8, center visual is at Z=0.4
    # The QR plate is at Z=0.3 (matching camera height roughly)
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{station_id}">
    <static>true</static>
    <link name="link">
      <!-- Support Pillar -->
      <collision name="pillar_collision">
        <pose>0 0 0.4 0 0 0</pose>
        <geometry>
          <box>
            <size>0.1 0.1 0.8</size>
          </box>
        </geometry>
      </collision>
      <visual name="pillar_visual">
        <pose>0 0 0.4 0 0 0</pose>
        <geometry>
          <box>
            <size>0.1 0.1 0.8</size>
          </box>
        </geometry>
        <material>
          <ambient>0.6 0.6 0.6 1</ambient>
          <diffuse>0.6 0.6 0.6 1</diffuse>
        </material>
      </visual>

      <!-- QR Code Plate -->
      <collision name="qr_collision">
        <pose>0.055 0 0.3 0 0 0</pose>
        <geometry>
          <box>
            <size>0.01 0.2 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="qr_visual">
        <pose>0.055 0 0.3 0 0 0</pose>
        <geometry>
          <box>
            <size>0.01 0.2 0.2</size>
          </box>
        </geometry>
        <material>
          <pbr>
            <metal>
              <albedo_map>materials/textures/{station_id}.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
    sdf_path = os.path.join(station_dir, 'model.sdf')
    with open(sdf_path, 'w') as f:
        f.write(sdf_content)
    print(f"Generated model SDF: {sdf_path}")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)
    output_dir = os.path.join(package_dir, 'models')
    
    stations = ['station_A_001', 'station_A_002', 'station_B_001', 'station_B_002']
    for station in stations:
        generate_station_model(station, output_dir)

if __name__ == '__main__':
    main()
