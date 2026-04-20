# program bekerja pakai sistem operasi windows dan linux, pastikan sudah terinstall python dan pip/pip3 di komputer anda
# jika library-nya belum terinstall, install dulu dengan perintah berikut di terminal: 
# windows: pip install matplotlib numpy
# linux: pip3 install matplotlib numpy
import matplotlib.pyplot as plt
import numpy as np

# deklarasi variabel tetap
Fcar = 6400  # Newton
murr = 0.015 # koefisien gesekan aspal
rho = 1.225 # kg/m^3
a = 2.2 # m^2
Cd = 0.3 # koefisien drag
m = 1250 # kg
g = 9.81 # m/s^2

# deklarasi variabel dinamis

slope = 0 # degree # derajat kemiringan jalan, bisa diubah sesuai kebutuhan

throtle = 100 # persen
v0 = 0 # m/s
t = np.linspace(0, 100, 1000) # sekon

# variabel hasil simulasi
v_sim = [v0] # kecepatan mobil pada setiap waktu
v_sim_PID = [v0] # kecepatan mobil dengan kontroler PID pada setiap waktu
v_setpoint = 20 # kecepatan target untuk kontroler PID
t_sim = [0] # waktu pada setiap iterasi
u_sim = [] # input PID pada setiap waktu

# fungsi percepatan mobil
def ace(v, t, u, theta):
    Fdrag = (rho*Cd*a*(v**2))/2
    Froll = murr*m*g*np.cos(theta)
    Fclimb = m*g*np.sin(theta)
    aceleration = (u - Fdrag - Froll - Fclimb)/m
    return aceleration

# fungsi kontroler untuk mengubah gaya dorong mobil
def PID_controller(v, t, setpoint):
    # PID controller dengan parameter yang disesuaikan berdasarkan kemiringan jalan
    Kp = 300 if slope >= 30 else 90 if slope >= 20 else 60 if slope >= 10 else 35
    Ki = 200 if slope >= 30 else 45 if slope >= 20 else 40 if slope >= 10 else 25
    Kd = 200 if slope >= 30 else 80 if slope >= 20 else 45 if slope >= 10 else 35

    error = setpoint - v
    integral = error * t
    derivative = (error - (setpoint - v_sim[-2])) / t if len(v_sim) > 1 else 0
    output = Kp * error + Ki * integral + Kd * derivative
    return output

# fungsi print setiap data
def pr(array1, array2, array3, array4):
    print(f'waktu\t\tv dengan\tu dengan\tv tanpa\t\tu tanpa\n{'='*71}')
    loop = 0
    lp = 0
    print(f'{array1[loop]}\t\t{(array2[loop]):.2f}\t\t{(array3[loop]):.2f}\t\t{(array4[loop]):.2f}\t\t{Fcar:.2f}')
    for i in array1:
        if lp == 10:
            print(f'{i:.2f}\t\t{(array2[loop]):.2f}\t\t{(array3[loop]):.2f}\t\t{(array4[loop]):.2f}\t\t{Fcar:.2f}')
            lp = 0
        lp += 1
        loop += 1


# simulasi akselerasi mobil
for i in range(1, len(t)):
    dt = t[i] - t[i-1]
    v = v_sim[-1] + ace(v_sim[-1], t[i], Fcar*throtle/100, np.radians(slope)) * dt 
    v_sim.append(v)
    t_sim.append(t[i])

# simulasi dengan kontroler PID untuk menjaga kecepatan tetap stabil
for i in range(1, len(t)):
    dt = t[i] - t[i-1]
    u = PID_controller(v_sim_PID[-1], t[i], v_setpoint) if PID_controller(v_sim_PID[-1], t[i], v_setpoint) <= Fcar else Fcar
    u_sim.append(u)
    v_pid = v_sim_PID[-1] + ace(v_sim_PID[-1], t[i], u, np.radians(slope)) * dt 
    v_sim_PID.append(v_pid)

# print hasil data yang disimulasikan
print(f'{pr(t_sim, v_sim_PID, u_sim, v_sim)}')

# visualisasi hasil simulasi
plt.style.use('bmh')
plt.plot(t_sim, v_sim_PID, color='blue', label='PID Control')
plt.plot(t_sim, v_sim, color='green', label='Without Control')
plt.legend([f'PID\nmax input = {max(u_sim):.2f} N\nmin input = {min(u_sim):.2f} N', f'Without PID\ninput = {Fcar*throtle/100} N'])
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.grid(True)
plt.title(f'Car Speed Simulation with {slope} Degree Slope')
plt.show()