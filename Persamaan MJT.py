import numpy as np # Mengimpor library numpy untuk operasi array dan operasi numerik
import time # Mengimpor library time untuk operasi waktu
import matplotlib.pyplot as plt # Mengimpor library matplotlib.pyplot untuk visualisasi data

class PathPlanTranslation(object): # Mendefinisikan kelas PathPlanTranslation
    def __init__(self, pose_init, pose_desired, total_time): # Metode konstruktor untuk inisialisasi objek
        self.position_init = pose_init[:3] # Mengambil tiga elemen pertama dari pose_init sebagai posisi awal (x, y, z)
        self.position_des = pose_desired[:3] # Mengambil tiga elemen pertama dari pose_desired sebagai posisi akhir (x, y, z)
        self.tfinal = total_time # Menyimpan waktu total perjalanan

    def trajectory_planning(self, t): # Metode untuk menghitung trajektori minimum jerk
        X_init = self.position_init[0] # Mengambil koordinat x dari posisi awal
        Y_init = self.position_init[1] # Mengambil koordinat y dari posisi awal
        Z_init = self.position_init[2] # Mengambil koordinat z dari posisi awal

        X_final = self.position_des[0] # Mengambil koordinat x dari posisi akhir
        Y_final = self.position_des[1] # Mengambil koordinat y dari posisi akhir
        Z_final = self.position_des[2] # Mengambil koordinat z dari posisi akhir

        # Menghitung posisi pada setiap sumbu (x, y, z) menggunakan persamaan minimum jerk trajectory
        x_traj = X_init + (X_final - X_init) * (10 * (t / self.tfinal)**3 - 15 * (t / self.tfinal)**4 + 6 * (t / self.tfinal)**5)
        y_traj = Y_init + (Y_final - Y_init) * (10 * (t / self.tfinal)**3 - 15 * (t / self.tfinal)**4 + 6 * (t / self.tfinal)**5)
        z_traj = Z_init + (Z_final - Z_init) * (10 * (t / self.tfinal)**3 - 15 * (t / self.tfinal)**4 + 6 * (t / self.tfinal)**5)
        position = np.array([x_traj, y_traj, z_traj]) # Membuat array posisi (x, y, z)
        
        vx = (X_final - X_init) * (30 * (t / self.tfinal)**2 - 60 * (t / self.tfinal)**3 + 30 * (t / self.tfinal)**4) / self.tfinal
        vy = (Y_final - Y_init) * (30 * (t / self.tfinal)**2 - 60 * (t / self.tfinal)**3 + 30 * (t / self.tfinal)**4) / self.tfinal
        vz = (Z_final - Z_init) * (30 * (t / self.tfinal)**2 - 60 * (t / self.tfinal)**3 + 30 * (t / self.tfinal)**4) / self.tfinal
        velocity = np.array([vx, vy, vz]) # Membuat array kecepatan (vx, vy, vz)

        return [position, velocity] # Mengembalikan array posisi dan kecepatan

if __name__ == "__main__": # Blok kode utama yang akan dieksekusi
    pose_init = np.array([0.334010, -0.515243, 0.2760, 2.41052252419476, 0.9219877505542503, 0.22313174746991263]) # Mendefinisikan posisi awal
    pose_des = np.array([0.953948, -0.015090, 0.6758, 2.41052252419476, 0.9219877505542503, 0.22313174746991263]) # Mendefinisikan posisi akhir yang diinginkan
    tfinal = 2 # Mendefinisikan waktu total perjalanan

    trajectory = PathPlanTranslation(pose_init, pose_des, tfinal) # Membuat objek PathPlanTranslation dengan posisi awal, posisi akhir, dan waktu total perjalanan

    posx = [] # Daftar untuk menyimpan posisi x
    posy = [] # Daftar untuk menyimpan posisi y
    posz = [] # Daftar untuk menyimpan posisi z
    v_x = [] # Daftar untuk menyimpan kecepatan x
    v_y = [] # Daftar untuk menyimpan kecepatan y
    v_z = [] # Daftar untuk menyimpan kecepatan z
    time_range = [] # Daftar untuk menyimpan rentang waktu

    t_start = time.time() # Menyimpan waktu awal
    while time.time() - t_start < tfinal: # Melakukan iterasi selama waktu saat ini kurang dari waktu total perjalanan
        t_current = time.time() - t_start # Menghitung waktu saat ini relatif terhadap waktu awal
        [position, velocity] = trajectory.trajectory_planning(t_current) # Menghitung trajektori minimum jerk pada waktu saat ini

        # Menyimpan posisi dan kecepatan ke dalam daftar yang sesuai
        posx.append(position[0])
        posy.append(position[1])
        posz.append(position[2])
        v_x.append(velocity[0])
        v_y.append(velocity[1])
        v_z.append(velocity[2])

        time_range.append(t_current) # Menyimpan waktu

# Plotting menggunakan pyplot
plt.figure() # Membuat figure baru
plt.plot(time_range, posx, label='X position') # Memplot posisi x terhadap waktu dengan label 'X position'
plt.plot(time_range, posy, label='Y position') # Memplot posisi y terhadap waktu dengan label 'Y position'
plt.plot(time_range, posz, label='Z position') # Memplot posisi z terhadap waktu dengan label 'Z position'
plt.legend() # Menampilkan legenda
plt.grid() # Menampilkan grid pada plot
plt.ylabel('Position [m]') # Memberi label pada sumbu y sebagai 'Position [m]'
plt.xlabel('Time [s]') # Memberi label pada sumbu x sebagai 'Time [s]'

plt.figure() # Membuat figure baru
plt.plot(time_range, v_x, label='X velocity') # Memplot kecepatan x terhadap waktu dengan label 'X velocity'
plt.plot(time_range, v_y, label='Y velocity') # Memplot kecepatan y terhadap waktu dengan label 'Y velocity'
plt.plot(time_range, v_z, label='Z velocity') # Memplot kecepatan z terhadap waktu dengan label 'Z velocity'
plt.legend() # Menampilkan legenda
plt.grid() # Menampilkan grid pada plot
plt.ylabel('Velocity[m/s]') # Memberi label pada sumbu y sebagai 'Velocity[m/s]' 
plt.xlabel('Time [s]') # Memberi label pada sumbu x sebagai 'Time [s]'

plt.show() # Menampilkan plot
