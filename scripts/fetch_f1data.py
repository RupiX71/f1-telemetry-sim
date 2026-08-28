import fastf1
import numpy as np
import pandas as pd
import os

print("Initializing FastF1...")

if not os.path.exists('cache'):
    os.makedirs("cache")
fastf1.Cache.enable_cache('cache')  # Enable caching to speed up data retrieval

# We will be using the 2025 Monza Grand Prix as the first example since it 
# is the most straight race and has fewer corners, which makes it easier to analyze the energy usage of the cars.
session = fastf1.get_session(2025, 'Monza', 'Q')  # Get the race session
session.load(telemetry=True, weather=False, messages=False)  # Load telemetry data

lap = session.laps.pick_fastest()  # Get the fastest lap of the session
tel = lap.get_telemetry()  # Get the telemetry data for the fastest lap
print(f"Telemetry data loaded successfully. {lap['Driver']} set the fastest lap with a time of {lap['LapTime']}.")

df = pd.DataFrame({
    'Distance': tel['Distance'], # Distance covered in meters
    'X': tel['X'], # X coordinate of the car on the track
    'Y': tel['Y'], # Y coordinate of the car on the track
    'Z': tel['Z'], # Z coordinate of the car on the track
})

# Get the radius of the corners using the X and Y coordinates of the car on the track
dx = np.gradient(df['X'])
dy = np.gradient(df['Y'])
ddx = np.gradient(dx)
ddy = np.gradient(dy)

denominator = (dx**2 + dy**2)**(1.5) + 1e-8 # Add a small value to avoid division by zero

curvature = np.abs(dx * ddy - dy * ddx) / denominator

# Radius of curvature is the inverse of curvature, 
# but we will set a maximum value of 10000 for straight lines (curvature close to zero)
df['Radius'] = np.where(curvature > 0.001, 1/ curvature, 10000)

# Smoothing
df['Radius'] = df['Radius'].rolling(window=15, min_periods=1, center=True).mean()


df['Segment_Length'] = df['Distance'].diff().fillna(df['Distance'].iloc[0])

if not os.path.exists('../data'):
    os.makedirs('../data')

output_path = '../data/monza_pole.csv'
df[['Segment_Length', 'Radius', 'X', 'Y']].to_csv(output_path, index=False)

print(f"Success! Exported {len(df)} segments to {output_path}")