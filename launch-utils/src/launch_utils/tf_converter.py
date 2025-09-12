import xml.etree.ElementTree as ET


class JsonToUrdf:
    @staticmethod
    def dict_to_origin(data):
        origin = ET.Element("origin")
        xyz = " ".join(map(str, data.get("xyz", [0, 0, 0])))
        rpy = " ".join(map(str, data.get("rpy", [0, 0, 0])))
        origin.set("xyz", xyz)
        origin.set("rpy", rpy)
        return origin

    @staticmethod
    def dict_to_geometry(data):
        geom = ET.Element("geometry")
        gtype = data["type"]
        if gtype == "box":
            box = ET.SubElement(geom, "box")
            box.set("size", " ".join(map(str, data["size"])))
        elif gtype == "cylinder":
            cyl = ET.SubElement(geom, "cylinder")
            cyl.set("radius", str(data["radius"]))
            cyl.set("length", str(data["length"]))
        elif gtype == "sphere":
            sph = ET.SubElement(geom, "sphere")
            sph.set("radius", str(data["radius"]))
        elif gtype == "mesh":
            mesh = ET.SubElement(geom, "mesh")
            mesh.set("filename", data["filename"])
        return geom

    @staticmethod
    def dict_to_inertial(data):
        inertial = ET.Element("inertial")
        if "origin" in data:
            inertial.append(JsonToUrdf.dict_to_origin(data["origin"]))
        mass = ET.SubElement(inertial, "mass")
        mass.set("value", str(data["mass"]))
        inertia = ET.SubElement(inertial, "inertia")
        for k, v in data["inertia"].items():
            inertia.set(k, str(v))
        return inertial

    @staticmethod
    def dict_to_visual_or_collision(tag_name, data):
        """Handles either a single dict or a list of dicts"""
        elems = []
        if isinstance(data, dict):
            data = [data]
        for entry in data:
            elem = ET.Element(tag_name)
            if "origin" in entry:
                elem.append(JsonToUrdf.dict_to_origin(entry["origin"]))
            elem.append(JsonToUrdf.dict_to_geometry(entry["geometry"]))
            elems.append(elem)
        return elems

    @staticmethod
    def dict_to_joint(jname, jdata):
        joint = ET.Element("joint", name=jname, type=jdata["type"])
        parent = ET.SubElement(joint, "parent")
        parent.set("link", jdata["parent"])
        child = ET.SubElement(joint, "child")
        child.set("link", jdata["child"])

        if "origin" in jdata:
            joint.append(JsonToUrdf.dict_to_origin(jdata["origin"]))

        if "axis" in jdata:
            axis = ET.SubElement(joint, "axis")
            axis.set("xyz", " ".join(map(str, jdata["axis"])))

        if "limit" in jdata:
            limit = ET.SubElement(joint, "limit")
            for k, v in jdata["limit"].items():
                limit.set(k, str(v))

        if "dynamics" in jdata:
            dynamics = ET.SubElement(joint, "dynamics")
            for k, v in jdata["dynamics"].items():
                dynamics.set(k, str(v))

        return joint

def json_to_urdf(json_data : dict):
    root = json_data['robot_description'] if 'robot_description' in json_data else json_data
    if 'name' not in root or 'structure' not in root:
        return ''
    robot = ET.Element("robot", name=root["name"])
    structure = root["structure"]
    assets = root.get("assets", {})

    # Links
    for lname in structure["links"]:
        link = ET.SubElement(robot, "link", name=lname)
        if lname in assets:
            ldata = assets[lname]

            if "inertial" in ldata:
                link.append(JsonToUrdf.dict_to_inertial(ldata["inertial"]))

            if "visual" in ldata:
                for v in JsonToUrdf.dict_to_visual_or_collision("visual", ldata["visual"]):
                    link.append(v)

            if "collision" in ldata:
                for c in JsonToUrdf.dict_to_visual_or_collision("collision", ldata["collision"]):
                    link.append(c)

    # Joints
    for jname, jdata in structure["joints"].items():
        joint = JsonToUrdf.dict_to_joint(jname, jdata)
        robot.append(joint)

    return ET.tostring(robot, encoding="unicode")

# if __name__ == "__main__":
#     with open("robot.json") as f:
#         data = json.load(f)
#     urdf_str = json_to_urdf(data)
#     with open("robot.urdf", "w") as f:
#         f.write(urdf_str)



class UrdfToJson:
    @staticmethod
    def parse_origin(elem):
        if elem is None:
            return {}
        xyz = elem.attrib.get("xyz", "0 0 0").split()
        rpy = elem.attrib.get("rpy", "0 0 0").split()
        return {
            "xyz": [float(x) for x in xyz],
            "rpy": [float(r) for r in rpy]
        }

    @staticmethod
    def parse_inertial(elem):
        if elem is None:
            return None
        inertial = {}
        origin = elem.find("origin")
        if origin is not None:
            inertial["origin"] = UrdfToJson.parse_origin(origin)
        mass_elem = elem.find("mass")
        if mass_elem is not None:
            inertial["mass"] = float(mass_elem.attrib["value"])
        inertia_elem = elem.find("inertia")
        if inertia_elem is not None:
            inertial["inertia"] = {k: float(v) for k, v in inertia_elem.attrib.items()}
        return inertial

    @staticmethod
    def parse_geometry(elem):
        if elem is None:
            return None
        if elem.find("box") is not None:
            size = [float(x) for x in elem.find("box").attrib["size"].split()]
            return {"type": "box", "size": size}
        if elem.find("cylinder") is not None:
            at = elem.find("cylinder").attrib
            return {"type": "cylinder", "radius": float(at["radius"]), "length": float(at["length"])}
        if elem.find("sphere") is not None:
            return {"type": "sphere", "radius": float(elem.find("sphere").attrib["radius"])}
        if elem.find("mesh") is not None:
            return {"type": "mesh", "filename": elem.find("mesh").attrib["filename"]}
        return None

    @staticmethod
    def parse_visual_or_collision(elems):
        result = []
        for elem in elems:
            entry = {}
            origin = elem.find("origin")
            if origin is not None:
                entry["origin"] = UrdfToJson.parse_origin(origin)
            geometry = UrdfToJson.parse_geometry(elem.find("geometry"))
            if geometry:
                entry["geometry"] = geometry
            result.append(entry)
        if not result:
            return None
        if len(result) == 1:
            return result[0]  # keep simple form if only one
        return result

    @staticmethod
    def parse_joint(elem):
        jdata = {
            "type": elem.attrib["type"],
            "parent": elem.find("parent").attrib["link"],
            "child": elem.find("child").attrib["link"]
        }
        origin = elem.find("origin")
        if origin is not None:
            jdata["origin"] = UrdfToJson.parse_origin(origin)
        axis = elem.find("axis")
        if axis is not None:
            jdata["axis"] = [float(x) for x in axis.attrib["xyz"].split()]
        limit = elem.find("limit")
        if limit is not None:
            jdata["limit"] = {k: float(v) for k, v in limit.attrib.items()}
        dynamics = elem.find("dynamics")
        if dynamics is not None:
            jdata["dynamics"] = {k: float(v) for k, v in dynamics.attrib.items()}
        return jdata

def urdf_to_json(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    robot_name = root.attrib["name"]
    structure = {"links": [], "joints": {}}
    assets = {}

    for elem in root:
        if elem.tag == "link":
            lname = elem.attrib["name"]
            structure["links"].append(lname)
            link_assets = {}
            inertial = UrdfToJson.parse_inertial(elem.find("inertial"))
            if inertial:
                link_assets["inertial"] = inertial
            visuals = elem.findall("visual")
            vis_data = UrdfToJson.parse_visual_or_collision(visuals)
            if vis_data:
                link_assets["visual"] = vis_data
            collisions = elem.findall("collision")
            col_data = UrdfToJson.parse_visual_or_collision(collisions)
            if col_data:
                link_assets["collision"] = col_data
            if link_assets:
                assets[lname] = link_assets

        elif elem.tag == "joint":
            jname = elem.attrib["name"]
            structure["joints"][jname] = UrdfToJson.parse_joint(elem)

    return {
        "robot": {
            "name": robot_name,
            "structure": structure,
            "assets": assets
        }
    }

# if __name__ == "__main__":
#     urdf_file = "robot.urdf"
#     json_out = urdf_to_json(urdf_file)
#     with open("robot.json", "w") as f:
#         json.dump(json_out, f, indent=2)

