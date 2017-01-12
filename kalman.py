import matplotlib
import numpy as np
import matplotlib.pyplot as plt

# Kalman Filter
# -------------------------
# k-1 = Time of current state
# k   = Time of next state
# x   = State [position, velocity]

# Prediction parameters
# ---------------------
# P = Covariance matrix (Sigma)
# F = Prediction matrix
# x_hat = Prediction
# u = Control vector
# B = Control matrix
# Q = Noise and external disturbance uncertanty

# Update parameters
# ---------------------
# H = Sensor model matrix
# mu_exp = Expected mean measurement
# sig_exp = Expected covariance in measurement
# R = Sensor noise covariance
# z = Sensor measurement mean
# K = Kalman gain


# Initial conditions
# ---------------------
k = 0           	# Time
n = 50			# Number of iternations
dt = 0.1		# Delta time between updates

# X_0
p_0 = 0.0		# Initial position
v_0 = 1.0		# Initial velocity
x = np.zeros( (n,2) )	# Initial State
x[0,:] = [p_0, v_0]

P = np.zeros( (n,2,2) )	# Covariance matrix 

F = [[1.0, dt],		# Prediction matrix
     [0.0, 1.0]]

u = 9.81		# Control vector
B = [dt**2, dt]		# Control matrix

Q = [[1e-5, 1e-5],	# Noise/External disturbance uncertanty
     [1e-5, 1e-5]]

H = [[1.0, 0.0],	# Sensor Model
     [0.0, 1.0]] 

R = np.zeros( (n,2,2) )	# Sensor noise
z = np.zeros( (n,2) )	# Sensor measurement noise

K = np.ones( (n,2,2) )  # Kalman gain

for k in range(1, n):
	
	print ("Filter Step: %d" % k)
	
	# Predict
	# ------------------------------------------
	x[k] = np.dot(F, x[k-1]) + np.dot(B,u)	
	print "xhat: ",
	print x[k]
	
	P[k] = np.dot(np.dot(F,P[k-1]),np.transpose(F)) + Q
	print "Phat: ",
	print P[k]

	# Update
	# ------------------------------------------
	z[k] = np.dot(H, x[k]) 			 # Ideal Measurement
	z[k] = z[k] + 5*(np.random.rand(1)-0.5)  # Added noise	
	print "Sensor mean: ",
	print z[k]

	R[k] = [[0.1**2, 0.1**2],		 # Measurement Noise
		[0.1**2, 0.1**2]]
	print "Sensor Cov: ",
	print R[k]	
		
	K[k] = np.dot(P[k], np.transpose(H))/( np.dot(np.dot(H, P[k]), np.transpose(H)) + R[k] )
	print "Kalman Gain: ",
	print K[k]

	x[k] = x[k] + np.dot(K[k], (z[k] - np.dot(H, x[k])))
	print "x: ",
	print x[k]

	P[k] = P[k] - np.dot(K[k], np.dot(H, P[k]))
	print "P: ",
	print P[k]

	print " "

plt.figure()
plt.plot(z[:,0], 'k+', label='Measurement Position')
plt.plot(z[:,1], 'k+', label='Measurement Velocity')
plt.plot(x[:,0], '-', label='Kalman Position')
plt.plot(x[:,1], '-', label='Kalman Velocity')
plt.legend()
plt.title('State Estimation vs Time', fontweight='bold')
plt.xlabel('Time')
plt.ylabel('State')
plt.show()

