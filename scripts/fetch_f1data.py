import os
import fastf1
import numpy as np
import pandas as pd
from scipy.interpolate import pchip_interpolate

print("Initializing FastF1...")

if not os.path.exists("cache"):
  os.makedirs("cache")
fastf1.Cache.enable_cache("cache")

# Carrega a sessão de Monza 2025
session = fastf1.get_session(2025, "Monza", "Q")
session.load(telemetry=True, weather=False, messages=False)

lap = session.laps.pick_fastest()
tel = lap.get_telemetry()
print(
    f"Telemetry loaded. {lap['Driver']} set the fastest lap: {lap['LapTime']}."
)

# --- 1. MELHORAR DEFINIÇÃO DOS SEGMENTOS (Interpolação Espacial) ---
# Criamos uma nova grade de distância perfeita (ex: de 1 em 1 metro)
distancia_original = tel["Distance"].to_numpy()
total_distance = distancia_original[-1]
# Segmentos fixos de 1 metro (mude para 0.5 ou 2 se quiser mais/menos pontos)
distancia_uniforme = np.arange(0, total_distance, 1.0)

# Interpolamos X, Y, Z com base na nova distância uniforme
# Usamos 'pchip' (Piecewise Cubic Hermite Interpolating Polynomial) pois preserva melhor as curvas que a linear
x_interp = pchip_interpolate(distancia_original, tel["X"].to_numpy(), distancia_uniforme)
y_interp = pchip_interpolate(distancia_original, tel["Y"].to_numpy(), distancia_uniforme)
z_interp = pchip_interpolate(distancia_original, tel["Z"].to_numpy(), distancia_uniforme)

df = pd.DataFrame({
    "Distance": distancia_uniforme,
    "X": x_interp,
    "Y": y_interp,
    "Z": z_interp,
})

# --- 2. ROTAÇÃO OFICIAL PERFEITA ---
circuit_info = session.get_circuit_info()
# Converte o ângulo oficial de graus para radianos (e inverte o sinal para alinhar com o padrão matemático)
angle_rad = -circuit_info.rotation / 180 * np.pi

cos_theta = np.cos(angle_rad)
sin_theta = np.sin(angle_rad)

# Aplicação da rotação correta
x_rot = df["X"] * cos_theta - df["Y"] * sin_theta
y_rot = df["X"] * sin_theta + df["Y"] * cos_theta

df["X"] = x_rot
df["Y"] = y_rot

# --- 3. CÁLCULO DO RAIO DE CURVATURA ---
# Como a distância agora é constante (1m), o gradient fica muito mais preciso
dx = np.gradient(df["X"])
dy = np.gradient(df["Y"])
ddx = np.gradient(dx)
ddy = np.gradient(dy)

denominator = (dx**2 + dy**2) ** 1.5 + 1e-8
curvature = np.abs(dx * ddy - dy * ddx) / denominator

# Define raio máximo para retas
df["Radius"] = np.where(curvature > 1e-3, 1 / curvature, 10000)

# Suavização leve no raio para evitar picos abruptos em mudanças de direção
df["Radius"] = (
    df["Radius"].rolling(window=5, min_periods=1, center=True).mean()
)

# Tamanho do segmento (agora será sempre ~1.0 devido à interpolação)
df["Segment_Length"] = df["Distance"].diff().fillna(df["Distance"].iloc[0])

# --- 4. EXPORTAÇÃO ---
if not os.path.exists("../data"):
  os.makedirs("../data")

output_path = "../data/monza_pole.csv"
df[["Segment_Length", "Radius", "X", "Y"]].to_csv(output_path, index=False)

print(f"Success! Exported {len(df)} segments to {output_path}")