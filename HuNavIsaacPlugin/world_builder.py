import omni.usd
import os

class WorldBuilder:
    def __init__(self, base_path):
        self.base_path = base_path
        self.usd_context = omni.usd.get_context()

    def load_map(self, map_name):
        map_path = f"{self.base_path}/scenarios/{map_name}.usd"
        if os.path.exists(map_path):
            self.usd_context.open_stage(map_path)
            print(f"Mapa {map_name} cargado desde {map_path}")
        else:
            print(f"Error: El mapa {map_name} no se pudo encontrar {map_path}")