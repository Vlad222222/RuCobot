#!/usr/bin/env python3
import xml.etree.ElementTree as ET

tree = ET.parse('rucobot_zero_mass_inertia.urdf')
root = tree.getroot()

for link in root:
	if link.tag == 'link':
		for inertial in link:
			if inertial.tag == 'inertial':
				#mass = inertial.find("mass")
				#print(mass.attrib)
				#mass.set('value', '0.1')

				inertia = inertial.find("inertia")
				#print(inertia.attrib)
				inertia.set('ixx', '0.0')
				inertia.set('ixy', '0.0')
				inertia.set('ixz', '0.0')
				inertia.set('iyy', '0.0')
				inertia.set('iyz', '0.0')
				inertia.set('izz', '0.0')

print('Done')
tree.write('rucobot_zero_mass_inertia.urdf')
