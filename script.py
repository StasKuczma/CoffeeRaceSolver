import requests
import numpy as np
import time
import folium
import polyline
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import gpxpy
import gpxpy.gpx
import csv

class TSPSolver:
    def __init__(self, addresses=None, transport_mode="driving", start_address=None, end_address=None):
        self.transport_mode = transport_mode
        self.addresses = addresses or []
        self.start_address = start_address
        self.end_address = end_address
        self.locations = []
        self.distance_matrix = None
        self.optimal_route = None
        self.osrm_url = "https://router.project-osrm.org"
        self.nominatim_url = "https://nominatim.openstreetmap.org/search"
        self.headers = {"User-Agent": "TSP-Solver-Script/1.0"}

    def add_address(self, address):
        self.addresses.append(address)

    def geocode_address(self, address):
        time.sleep(1)
        params = {'q': address, 'format': 'json', 'limit': 1}
        try:
            response = requests.get(self.nominatim_url, params=params, headers=self.headers)
            if response.status_code == 200:
                data = response.json()
                if data:
                    return {
                        'address': address,
                        'lat': float(data[0]['lat']),
                        'lon': float(data[0]['lon']),
                        'display_name': data[0]['display_name']
                    }
        except Exception as e:
            print(f"Error geocoding {address}: {e}")
        return None

    def geocode_all_addresses(self):
        self.locations = []
        all_addresses = self.addresses.copy()

        if self.start_address:
            all_addresses = [self.start_address] + [a for a in all_addresses if a != self.start_address]

        if self.end_address and self.end_address not in all_addresses:
            all_addresses.append(self.end_address)

        for addr in all_addresses:
            result = self.geocode_address(addr)
            if result:
                self.locations.append(result)

    def build_distance_matrix(self):
        coords = ";".join(f"{loc['lon']},{loc['lat']}" for loc in self.locations)
        url = f"{self.osrm_url}/table/v1/{self.transport_mode}/{coords}"
        params = {'annotations': 'duration'}

        time.sleep(1)
        try:
            response = requests.get(url, params=params)
            if response.status_code == 200:
                self.distance_matrix = np.array(response.json()['durations'])
        except Exception as e:
            print(f"Error fetching distance matrix: {e}")

    def solve_tsp(self):
        if self.distance_matrix is None:
            return None
        n = len(self.locations)
        distance_matrix_int = (self.distance_matrix * 100).astype(int)

        start_index = 0
        end_index = 0
        if self.start_address:
            start_index = next(i for i, loc in enumerate(self.locations) if loc['address'] == self.start_address)
        if self.end_address:
            end_index = next(i for i, loc in enumerate(self.locations) if loc['address'] == self.end_address)

        manager = pywrapcp.RoutingIndexManager(n, 1, [start_index], [end_index])
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            return distance_matrix_int[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        search_parameters.time_limit.seconds = 30

        solution = routing.SolveWithParameters(search_parameters)
        if solution:
            route = []
            index = routing.Start(0)
            while not routing.IsEnd(index):
                route.append(manager.IndexToNode(index))
                index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))  

            total_duration = sum(self.distance_matrix[route[i]][route[i + 1]] for i in range(len(route) - 1))
            self.optimal_route = {
                'route_indices': route,
                'route_locations': [self.locations[i] for i in route],
                'total_duration': total_duration,
                'total_duration_formatted': self.format_duration(total_duration)
            }
            return self.optimal_route

    def get_route_directions(self, route_indices=None):
        if route_indices is None and self.optimal_route:
            route_indices = self.optimal_route['route_indices']
        if not route_indices:
            return None, 0
        directions = []
        total_distance = 0
        for i in range(len(route_indices) - 1):
            from_loc = self.locations[route_indices[i]]
            to_loc = self.locations[route_indices[i + 1]]
            coords = f"{from_loc['lon']},{from_loc['lat']};{to_loc['lon']},{to_loc['lat']}"
            url = f"{self.osrm_url}/route/v1/{self.transport_mode}/{coords}"
            params = {'steps': 'true', 'overview': 'full', 'geometries': 'polyline'}
            time.sleep(1)
            try:
                response = requests.get(url, params=params)
                if response.status_code == 200:
                    data = response.json()
                    if data['routes']:
                        route_data = data['routes'][0]
                        total_distance += route_data['distance']  # meters
                        directions.append({
                            'from': from_loc['address'],
                            'to': to_loc['address'],
                            'geometry': route_data['geometry']
                        })
            except Exception as e:
                print(f"Error getting segment from {from_loc['address']} to {to_loc['address']}: {e}")
        return directions, total_distance

    def create_map(self, save_to_file="route.html"):
        if not self.optimal_route:
            return None
        directions, total_distance = self.get_route_directions()
        self.optimal_route['total_distance_m'] = total_distance
        self.optimal_route['total_distance_km'] = round(total_distance / 1000, 2)
        print(f"Total route distance: {self.optimal_route['total_distance_km']} km")
        m = folium.Map(location=[self.locations[0]['lat'], self.locations[0]['lon']], zoom_start=13)
        for i, loc in enumerate(self.optimal_route['route_locations']):
            label = f"{i + 1}. {loc['address']}"
            folium.Marker([loc['lat'], loc['lon']], popup=label).add_to(m)
        for seg in directions:
            folium.PolyLine(polyline.decode(seg['geometry']), color='blue').add_to(m)
        m.save(save_to_file)
        print(f"Map saved as {save_to_file}")
        return m

    def save_route_to_txt(self, filename="order.txt"):
        if self.optimal_route:
            with open(filename, "w", encoding="utf-8") as f:
                f.write("Optimal Route Order:\n")
                for i, loc in enumerate(self.optimal_route['route_locations']):
                    f.write(f"{i + 1}. {loc['address']} ({loc['display_name']})\n")
                f.write(f"\nTotal Duration: {self.optimal_route['total_duration_formatted']}\n")
                f.write(f"Total Distance: {self.optimal_route['total_distance_km']} km\n")
            print(f"Route order saved to {filename}")

    def save_route_to_gpx(self, filename="route.gpx"):
        if not self.optimal_route:
            print("No route found.")
            return
        directions, _ = self.get_route_directions()

        gpx = gpxpy.gpx.GPX()
        gpx_track = gpxpy.gpx.GPXTrack()
        gpx.tracks.append(gpx_track)
        gpx_segment = gpxpy.gpx.GPXTrackSegment()
        gpx_track.segments.append(gpx_segment)

        for segment in directions:
            coords = polyline.decode(segment['geometry'])
            for lat, lon in coords:
                gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(latitude=lat, longitude=lon))

        with open(filename, "w") as f:
            f.write(gpx.to_xml())
        print(f"Route saved as {filename}")

    def format_duration(self, seconds):
        h, rem = divmod(seconds, 3600)
        m, s = divmod(rem, 60)
        return f"{int(h)}h {int(m)}m {int(s)}s"


if __name__ == "__main__":
    addresses = []

    with open('addresses.csv', mode='r', encoding='utf-8') as file:
        reader = csv.DictReader(file)
        for row in reader:
            address = f"{row['Street']}, {row['PostalCode']} {row['City']}"
            addresses.append(address)

    start = "Śródka 1, 61-125 Poznań"
    end = "Wierzbięcice 10/2a, 61-568 Poznań"
    mode = "cycling"

    tsp = TSPSolver(addresses=addresses, transport_mode=mode, start_address=start, end_address=end)
    print("Geocoding...")
    tsp.geocode_all_addresses()
    print("Building distance matrix...")
    tsp.build_distance_matrix()
    print("Solving TSP...")
    tsp.solve_tsp()
    print("Saving map and route...")
    tsp.create_map()
    tsp.save_route_to_txt()
    tsp.save_route_to_gpx()

