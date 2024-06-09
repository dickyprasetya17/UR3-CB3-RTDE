import sys # Mengimpor modul sys
sys.path.append('') # Menambahkan direktori kerja saat ini ke dalam sys.path
import logging # Mengimpor modul logging
import rtde as rtde # Mengimpor modul rtde
from ur_rtde import rtde_config # Mengimpor kelas rtde_config dari modul ur_rtde
import time # Mengimpor modul time
from matplotlib import pyplot as plt # Mengimpor modul pyplot dari paket matplotlib
from min_jerk_planner_translation import PathPlanTranslation # Mengimpor kelas PathPlanTranslation dari modul min_jerk_planner_translation

# -------- functions -------------
def setp_to_list(setp): # Fungsi untuk mengonversi objek setp menjadi daftar
    temp = [] # Inisialisasi daftar kosong
    for i in range(0, 6): # Loop dari 0 hingga 5
        temp.append(setp.__dict__["input_double_register_%i" % i]) # Menambahkan nilai input_double_register_i ke daftar temp
    return temp # Mengembalikan daftar temp

def list_to_setp(setp, list): # Fungsi untuk mengonversi daftar menjadi objek setp
    for i in range(0, 6): # Loop dari 0 hingga 5
        setp.__dict__["input_double_register_%i" % i] = list[i] # Mengatur nilai input_double_register_i dari daftar yang diberikan
    return setp # Mengembalikan objek setp

# ------------- robot communication stuff -----------------
ROBOT_HOST = '10.29.202.104' # Alamat IP robot
ROBOT_PORT = 30004 # Port robot
config_filename = r'C:\Users\dicky\Downloads\Compressed\RTDE_Python_Client_Library-main\Servoj_RTDE_UR3-main\control_loop_configuration.xml' # Lokasi file konfigurasi XML

logging.getLogger().setLevel(logging.INFO) # Mengatur level logging ke INFO

conf = rtde_config.ConfigFile(config_filename) # Membaca file konfigurasi XML
state_names, state_types = conf.get_recipe('state') # Mendapatkan daftar nama dan tipe data untuk mengakses keadaan robot
setp_names, setp_types = conf.get_recipe('setp') # Mendapatkan daftar nama dan tipe data untuk mengakses input robot
watchdog_names, watchdog_types= conf.get_recipe('watchdog') # Mendapatkan daftar nama dan tipe data untuk mengakses watchdog

# -------------------- Establish connection--------------------
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT) # Membuat objek RTDE dengan alamat IP dan port robot
connection_state = con.connect() # Mencoba menghubungkan ke robot

# check if connection has been established
while connection_state != 0: # Loop hingga koneksi berhasil
    time.sleep(0.5) # Tunggu 0,5 detik
    connection_state = con.connect() # Coba menghubungkan kembali
print("---------------Successfully connected to the robot-------------\n") # Cetak pesan konfirmasi koneksi

# get controller version
con.get_controller_version() # Mendapatkan versi kontroler robot

# ------------------- setup recipes ----------------------------
FREQUENCY = 500 # Frekuensi pengiriman data (500 Hz)
con.send_output_setup(state_names, state_types, FREQUENCY) # Mengatur paket output (state) dengan frekuensi yang diberikan
setp = con.send_input_setup(setp_names, setp_types) # Mengatur paket input (setp) untuk dikirim ke robot
watchdog = con.send_input_setup(watchdog_names, watchdog_types) # Mengatur paket input (watchdog) untuk dikirim ke robot

setp.input_double_register_0 = 0 # Menginisialisasi nilai input_double_register_0 dengan 0
setp.input_double_register_1 = 0 # Menginisialisasi nilai input_double_register_1 dengan 0
setp.input_double_register_2 = 0 # Menginisialisasi nilai input_double_register_2 dengan 0
setp.input_double_register_3 = 0 # Menginisialisasi nilai input_double_register_3 dengan 0
setp.input_double_register_4 = 0 # Menginisialisasi nilai input_double_register_4 dengan 0
setp.input_double_register_5 = 0 # Menginisialisasi nilai input_double_register_5 dengan 0

setp.input_bit_registers0_to_31 = 0 # Menginisialisasi nilai input_bit_registers0_to_31 dengan 0

watchdog.input_int_register_0 = 0 # Menginisialisasi nilai input_int_register_0 dengan 0

# start data synchronization
if not con.send_start(): # Jika gagal memulai sinkronisasi data
    sys.exit() # Keluar dari program

start_pose = [-0.3, -0.44, 0.1614904966558569, -0.1, -3.1, 0.1] # Posisi awal robot
desired_pose =  [-0.1, -0.35, 0.2, -0.1, -3.1, 0.1] # Posisi akhir yang diinginkan

orientation_const = start_pose[3:] # Mengambil nilai orientasi dari posisi awal

state = con.receive() # Menerima keadaan terkini dari robot
tcp1 = state.actual_TCP_pose # Mendapatkan posisi TCP saat ini
print(tcp1) # Mencetak posisi TCP saat ini

#   ------------  mode = 1 (Connection) -----------
while True: # Loop tak terhingga
    print('Boolean 1 is False, please click CONTINUE on the Polyscope') # Mencetak pesan untuk pengguna
    state = con.receive() # Menerima keadaan terkini dari robot
    con.send(watchdog) # Mengirim paket watchdog ke robot
    # print(f"runtime state is {state.runtime_state}") # Komentar untuk mencetak keadaan runtime
    if state.output_bit_registers0_to_31 == True: # Jika bit register 0-31 bernilai True
        print('Boolean 1 is True, Robot Program can proceed to mode 1\n') # Mencetak pesan
        break # Keluar dari loop
print("-------Executing moveL -----------\n") # Mencetak pesan

watchdog.input_int_register_0 = 1 # Mengatur nilai input_int_register_0 pada watchdog menjadi 1
con.send(watchdog) # Mengirim paket watchdog ke robot
list_to_setp(setp, start_pose) # Mengonversi start_pose menjadi objek setp
con.send(setp) # Mengirim paket setp ke robot

while True: # Loop tak terhingga
    print('Waiting for moveL() to finish') # Mencetak pesan
    state = con.receive() # Menerima keadaan terkini dari robot
    con.send(watchdog) # Mengirim paket watchdog ke robot
    if state.output_bit_registers0_to_31 == False: # Jika bit register 0-31 bernilai False
        print('Proceeding to mode 2\n') # Mencetak pesan
        break # Keluar dari loop
print("-------Executing moveL  -----------\n") # Mencetak pesan

watchdog.input_int_register_0 = 2 # Mengatur nilai input_int_register_0 pada watchdog menjadi 2
con.send(watchdog) # Mengirim paket watchdog ke robot

trajectory_time = 2 # Waktu untuk lintasan minimum jerk (2 detik)
dt = 1/500 # Interval waktu (1/500 Hz = 0,002 detik)
plotter = True # Mengatur flag plotter menjadi True

# ------------------ Control loop initialization -------------------------

planner = PathPlanTranslation(start_pose, desired_pose, trajectory_time) # Membuat objek planner untuk lintasan minimum jerk
# ----------- minimum jerk preparation -----------------------

if plotter: # Jika plotter diaktifkan
    time_plot = [] # Inisialisasi daftar kosong untuk menyimpan waktu

    min_jerk_x = [] # Inisialisasi daftar kosong untuk menyimpan posisi x minimum jerk
    min_jerk_y = [] # Inisialisasi daftar kosong untuk menyimpan posisi y minimum jerk
    min_jerk_z = [] # Inisialisasi daftar kosong untuk menyimpan posisi z minimum jerk

    min_jerk_vx = [] # Inisialisasi daftar kosong untuk menyimpan kecepatan x minimum jerk
    min_jerk_vy = [] # Inisialisasi daftar kosong untuk menyimpan kecepatan y minimum jerk
    min_jerk_vz = [] # Inisialisasi daftar kosong untuk menyimpan kecepatan z minimum jerk

    px = [] # Inisialisasi daftar kosong untuk menyimpan posisi x robot
    py = [] # Inisialisasi daftar kosong untuk menyimpan posisi y robot
    pz = [] # Inisialisasi daftar kosong untuk menyimpan posisi z robot

    vx = [] # Inisialisasi daftar kosong untuk menyimpan kecepatan x robot
    vy = [] # Inisialisasi daftar kosong untuk menyimpan kecepatan y robot
    vz = [] # Inisialisasi daftar kosong untuk menyimpan kecepatan z robot

#   -------------------------Control loop --------------------
state = con.receive() # Menerima keadaan terkini dari robot
tcp = state.actual_TCP_pose # Mendapatkan posisi TCP saat ini
t_current = 0 # Inisialisasi waktu saat ini menjadi 0
t_start = time.time() # Mendapatkan waktu mulai

while time.time() - t_start < trajectory_time: # Loop selama waktu belum mencapai waktu lintasan
    t_init = time.time() # Mendapatkan waktu saat ini
    state = con.receive() # Menerima keadaan terkini dari robot
    t_prev = t_current # Menyimpan waktu sebelumnya
    t_current = time.time() - t_start # Menghitung waktu saat ini

    print(f"dt:{t_current-t_prev}") # Mencetak selisih waktu antara waktu saat ini dan sebelumnya

    # read state from the robot
    if state.runtime_state > 1: # Jika keadaan runtime lebih besar dari 1
        #   ----------- minimum_jerk trajectory --------------
        if t_current <= trajectory_time: # Jika waktu saat ini kurang dari atau sama dengan waktu lintasan
            [position_ref, lin_vel_ref, acceleration_ref] = planner.trajectory_planning(t_current) # Merencanakan lintasan minimum jerk

        # ------------------ impedance -----------------------
        current_pose = state.actual_TCP_pose # Mendapatkan posisi TCP saat ini
        current_speed = state.actual_TCP_speed # Mendapatkan kecepatan TCP saat ini

        pose = position_ref.tolist() + orientation_const # Membuat daftar pose dengan menggabungkan posisi referensi dan orientasi konstan

        list_to_setp(setp, pose) # Mengonversi daftar pose menjadi objek setp
        con.send(setp) # Mengirim paket setp ke robot

        if plotter: # Jika plotter diaktifkan
            time_plot.append(time.time() - t_start) # Menambahkan waktu saat ini ke daftar time_plot

            min_jerk_x.append(position_ref[0]) # Menambahkan posisi x minimum jerk ke daftar min_jerk_x
            min_jerk_y.append(position_ref[1]) # Menambahkan posisi y minimum jerk ke daftar min_jerk_y
            min_jerk_z.append(position_ref[2]) # Menambahkan posisi z minimum jerk ke daftar min_jerk_z

            min_jerk_vx.append(lin_vel_ref[0]) # Menambahkan kecepatan x minimum jerk ke daftar min_jerk_vx
            min_jerk_vy.append(lin_vel_ref[1]) # Menambahkan kecepatan y minimum jerk ke daftar min_jerk_vy
            min_jerk_vz.append(lin_vel_ref[2]) # Menambahkan kecepatan z minimum jerk ke daftar min_jerk_vz

            px.append(current_pose[0]) # Menambahkan posisi x robot ke daftar px
            py.append(current_pose[1]) # Menambahkan posisi y robot ke daftar py
            pz.append(current_pose[2]) # Menambahkan posisi z robot ke daftar pz

            vx.append(current_speed[0]) # Menambahkan kecepatan x robot ke daftar vx
            vy.append(current_speed[1]) # Menambahkan kecepatan y robot ke daftar vy
            vz.append(current_speed[2]) # Menambahkan kecepatan z robot ke daftar vz

print(f"It took {time.time()-t_start} s to execute the moveL") # Mencetak waktu yang dibutuhkan untuk mengeksekusi moveL
print(f"time needed for min_jerk: {trajectory_time} s\n") # Mencetak waktu yang dibutuhkan untuk lintasan minimum jerk

state = con.receive() # Menerima keadaan terkini dari robot
print('--------------------\n') # Mencetak pemisah
print('Actual Current TCP Pose:\n') # Mencetak label
print(state.actual_TCP_pose) # Mencetak posisi TCP saat ini

# ====================mode 3===================
watchdog.input_int_register_0 = 3 # Mengatur nilai input_int_register_0 pada watchdog menjadi 3
con.send(watchdog) # Mengirim paket watchdog ke robot

con.send_pause() # Mengirim perintah pause ke robot
con.disconnect() # Memutuskan koneksi dengan robot

if plotter: # Jika plotter diaktifkan
    # ----------- position -------------
    plt.figure() # Membuat figure baru
    plt.plot(time_plot, min_jerk_x, label="x_min_jerk") # Memplot posisi x minimum jerk
    plt.plot(time_plot, px, label="x_robot") # Memplot posisi x robot
    plt.legend() # Menampilkan legenda
    plt.grid() # Menampilkan grid
    plt.ylabel('Position in x[m]') # Label sumbu y
    plt.xlabel('Time [sec]') # Label sumbu x

    plt