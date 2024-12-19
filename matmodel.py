import math
import numpy as np
import matplotlib.pyplot as plt
import json

#получения данных из KSP
data = {}
with open("data (7).json", "r") as f:
    data = json.load(f)


# Константы и параметры
G = 6.67430e-11
M_kerbin = 5.292e22  # Масса Кербина (кг)
R_kerbin = 600e3  # Радиус Кербина (м)
rho0 = 1.114  # Плотность воздуха на уровне моря (кг/м³)
H = 5600  # Характерная высота атмосферы (м)
d = 0.5  # Коэффициент сопротивления формы
A = 69.4 # Площадь поперечного сечения ракеты (м²)

t1 = 60 # Момент времени отсоединения первой ступени
t2 = 173 # Момент времени отсоединения второй ступени
t3 = 300 # Момент времени отсоединения второй ступени

# Исходные параметры ракеты
m0 = 296754.375  # Общая масса ракеты (с полезной нагрузкой и топливом)
m1 = 65280  # Сухая масса первой ступени
m2 = 40000
m3 = 30000 # Сухая масса второй ступени
F_t1 = 5390e3 # Сила тяги первой ступени (Н) (4 двигателя в сумме)
F_t2 = 1460e3  # Сила тяги второй ступени (Н) (1 центральный двигатель)
F_t3 = 215000 # Сила тяги третье ступени (Н) (1 двигатель)
delta1 = 1100 # Расход топлива первой ступени (кг/с)
delta2 = 500 # Расход топлива второй ступени (кг/с)
delta3 = 60 # Расход топлива третье ступени (кг/с)

# Начальные условия
v_x = 0  # Горизонтальная скорость (м/с)
v_y = 0  # Вертикальная скорость (м/с)
h = 0  # Высота (м)
current_mass = m0
V_fin = 0
t = 0
x = 0
y = R_kerbin

dt = 1 # Шаг времени (с)

time = []  # Массив времеи
speed_values = []  # Массив полной скорости
height_values = []  # Массив высоты
angel_values = []  # Массив полной скорости
mass_values = []  # Массив высоты
rho_values = []  # Массив полной скорости
power_values = []  # Массив высоты
vx_values = []  # Массив полной скорости
vy_values = []  # Массив высоты
x_values = [] # Массив значений по х
y_values = [] # Массив значений по y

turn_start_altitude = 250
turn_end_altitude = 150_000  # 150
target_altitude = 300_000
turn_angle = 0
target_time = 200


def M(t):
    if t <= t1:
        return m0 - delta1 * t - delta2 * t
    elif t > t1 and t <= t2:
        return m0 - delta1 * t1 - delta2 * t1 - m1 - delta2 * (t - t1)
    else:
        return m0 - delta1 * t1 - delta2 * t1 - m1 - delta2 * (t2 - t1) - m2 - delta3 * (t - t2)


def g(h):
    return G * M_kerbin / (R_kerbin + h) ** 2


def rho(h):
    return rho0 * (1 / (math.e ** (h / H)))


# Расчеты
while t < target_time :  # Общее время полета

    if turn_start_altitude <= h <= turn_end_altitude:
        frac = ((h - turn_start_altitude) /
            (turn_end_altitude - turn_start_altitude))
        new_turn_angle = frac * 90
        turn_angle = new_turn_angle

    # Определение текущей тяги и расхода топлива
    if t <= t1:  # Первая ступень
        F_t = F_t1
    elif t1 < t <= t2:  # Вторая ступень
        F_t = F_t2
    else:
        F_t = F_t3

    if h == 300000:
        break

    cur_rho = rho(h)
    # Обновление скоростей по осям
    if V_fin < math.sqrt((G * M_kerbin) / (R_kerbin + target_altitude)):
        v_y += dt * ((F_t * math.sin(math.radians(90 - turn_angle)) - (current_mass * g(t)) - (0.5 * cur_rho * (V_fin ** 2) * math.sin(math.radians(90 - turn_angle)) * d * A)) / current_mass)
        v_x += dt * ((F_t * math.cos(math.radians(90 - turn_angle)) - (0.5 * cur_rho * (V_fin ** 2) * math.cos(math.radians(90 - turn_angle)) * d * A)) / current_mass)

    current_mass = M(t)
    x += v_x * dt
    y += v_y * dt
    h = math.sqrt(x ** 2 + y ** 2) - R_kerbin
    # Обновление полной скорости и высоты
    V_fin = math.sqrt(v_x ** 2 + v_y ** 2)

    # Запись данных для графиков
    time.append(t)
    speed_values.append(V_fin)
    height_values.append(h)
    angel_values.append(90 - turn_angle)
    mass_values.append(current_mass)
    rho_values.append(cur_rho)
    power_values.append(F_t)
    vx_values.append(v_x)
    vy_values.append(v_y)
    x_values.append(x)
    y_values.append(y)
    t += dt


# Построение графиков
fig = plt.figure(figsize=(20, 20))

#Получение данных из KSP
time2 = [i for i in data["time"] if i < target_time ]
mass = np.array([data["mass"][i] for i in range(len(time2))])
speed_surface = np.array([data["velocity_surface"][i] for i in range(len(time2))])
speed_orbit = np.array([data["velocity_orbit"][i] for i in range(len(time2))])
angle = np.array([data["angle"][i] for i in range(len(time2))])
height = np.array([data["altitude"][i] for i in range(len(time2))])
thrust = [data["thrust"][i] for i in range(len(time2))]
rho = [data["atmosphere_density"][i] for i in range(len(time2))]
pos_x = np.array([data["position"][i][0] for i in range(len(time2))])
pos_y = np.array([-data["position"][i][2] for i in range(len(time2))])
pos_z = np.array([-data["position"][i][1] for i in range(len(time2))])

# График полной скорости
plt.subplot(2, 3, 1)
plt.plot(time, speed_values,'red', label="Физ.модель")
plt.plot(time2, speed_surface,'blue', label="KSP")
plt.legend()
plt.title("График полной скорости ракеты")
plt.ylabel("Скорость (м/с)")
plt.grid()
# #
# # #График высоты
plt.subplot(2, 3, 2)
plt.plot(time, height_values, 'red', label="Физ.модель")
plt.plot(time2, height, 'blue', label="KSP")
plt.legend()
plt.title("График высоты ракеты")
plt.ylabel("Высота (м)")
plt.grid()
# # #
# # # График угла наклона
plt.subplot(2, 3, 3)
plt.plot(time, angel_values, 'red', label="Физ.модель")
plt.plot(time2, angle, 'blue', label="KSP")
plt.legend()
plt.title("График угла наклона ракеты")
plt.ylabel("Угол (градусы)")
plt.grid()
#
# График массы
plt.subplot(2, 3, 4)
plt.plot(time, mass_values, 'red', label="Физ.модель")
plt.plot(time2, mass, 'blue', label="KSP")
plt.legend()
plt.title("График массы ракеты")
plt.xlabel("Время (с)")
plt.ylabel("масса (кг)")
plt.grid()
#
#
# #График силы тяги
plt.subplot(2, 3, 5)
plt.plot(time, power_values, 'red', label="Физ.модель")
plt.plot(time2, thrust, 'blue', label="KSP")
plt.legend()
plt.title("График силы тяги ракеты")
plt.xlabel("Время (с)")
plt.ylabel("Сила (Н)")
plt.grid()
#
# #График плотности
plt.subplot(2, 3, 6)
plt.plot(time, rho_values, 'red',  label="Физ.модель",)
plt.plot(time2, rho, "blue", label="KSP")
plt.legend()
plt.title("График плотности атмосферы")
plt.xlabel("Время (с)")
plt.ylabel("Плотность (кг/м3)")
plt.grid()
#
#
# График y от x
figure1 = plt.figure(figsize=(5, 5))
ax = figure1.add_subplot()
draw_circle = plt.Circle((0, 0), R_kerbin,)
ax.add_artist(draw_circle)
ax.set(xlim=(-R_kerbin, R_kerbin), ylim=(0, 2 * R_kerbin))
plt.plot(x_values, y_values, "red", label="Физ.модель")
plt.plot(pos_x, pos_y, "blue", label="KSP")
plt.xlabel("м")
plt.ylabel("м")

plt.show()

