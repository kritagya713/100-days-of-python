import requests
import time
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# --- 1. CONFIGURATION ---
# Your verified Access Token
LOCATIONIQ_TOKEN = "pk.6b688df8b89f4ad93d0a41f543af0473"

def get_coordinates_pro(name):
    """Uses LocationIQ for high-precision Nepal landmark searching."""
    url = "https://us1.locationiq.com/v1/search.php"
    params = {
        'key': LOCATIONIQ_TOKEN,
        'q': f"{name}, Nepal",
        'format': 'json',
        'addressdetails': 1,
        'limit': 1
    }
    
    try:
        # We must stay under 2 requests per second for the free tier
        time.sleep(0.7) 
        response = requests.get(url, params=params, timeout=10)
        data = response.json()
        
        if isinstance(data, list) and len(data) > 0:
            res = data[0]
            # Check if it's actually in Nepal
            if "nepal" in res.get('display_name', '').lower():
                print(f"✅ Found: {res['display_name'][:60]}...")
                return (float(res['lat']), float(res['lon']))
        
        print(f"❌ Error: Could not find '{name}' in Nepal.")
    except Exception as e:
        print(f"📡 Connection Error: {e}")
    return None

def get_distance_matrix_bulk(coords):
    """Fetches real road distances using OSRM Table Service."""
    formatted = ";".join([f"{c[1]},{c[0]}" for c in coords])
    url = f"http://router.project-osrm.org/table/v1/driving/{formatted}?annotations=distance"
    try:
        r = requests.get(url, timeout=10).json()
        if r['code'] == 'Ok':
            return r['distances']
    except:
        return None

# --- 2. INPUT LOOP ---
print("==========================================")
print("   🇳🇵 GALLI-OPTIMIZER PRO: 2026 🇳🇵   ")
print("==========================================")
print("Enter locations (1st is START). Type 'done' to calculate.\n")

location_names = []
while True:
    stop = input(f"Location {len(location_names) + 1}: ").strip()
    if stop.lower() == 'done': break
    if stop: location_names.append(stop)

if len(location_names) < 3:
    print("❌ Please enter at least 3 locations!")
    exit()

# --- 3. PROCESSING ---
valid_coords = []
valid_names = []

print("\n[1/3] Locating points using LocationIQ...")
for name in location_names:
    pt = get_coordinates_pro(name)
    if pt:
        valid_coords.append(pt)
        valid_names.append(name)

if len(valid_coords) < 2:
    print("❌ Not enough valid locations found. Try being more specific.")
    exit()

print(f"[2/3] Fetching road network data...")
matrix = get_distance_matrix_bulk(valid_coords)

# --- 4. OR-TOOLS OPTIMIZATION ---
def solve_tsp(dist_matrix):
    manager = pywrapcp.RoutingIndexManager(len(dist_matrix), 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return int(dist_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    
    solution = routing.SolveWithParameters(params)
    if solution:
        index = routing.Start(0)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index)) # Return to start
        return route
    return None

print("[3/3] Solving Traveling Salesman Problem...")
order = solve_tsp(matrix)

# --- 5. FINAL REPORT ---
if order:
    print("\n" + "🚀 OPTIMIZED DELIVERY ROUTE " + "="*10)
    total_m = 0
    for i, node_idx in enumerate(order):
        label = "START/FINISH" if (i == 0 or i == len(order)-1) else f"STOP {i}"
        print(f"{label}: {valid_names[node_idx].upper()}")
        
        if i < len(order) - 1:
            total_m += matrix[order[i]][order[i+1]]

    total_km = total_m / 1000
    fuel_cost = (total_km / 40) * 202 # Rs. 202/L at 40km/l mileage
    
    print("-" * 40)
    print(f"🛣️  Total Distance: {total_km:.2f} km")
    print(f"💰 Total Fuel Cost: Rs. {fuel_cost:.2f}")
    print("=" * 40)
else:
    print("❌ Optimization failed. Check your locations.")