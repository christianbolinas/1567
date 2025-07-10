# z(t) = K_p e(t) + K_i \int_0^t e(t)dt + K_d de/dt
#z(t) := control value at time t == angular.z
#e(t) := error at time t == number of pixel of the center of the object to the center of the screen
#K_p, K_i, K_d := coefficient to be found through trial and error?
#set de/dt to e(t) - e(t-1) for simplicity 

et = 0.0
etprev = 0.0

#use tans center algorithm to find "center", make number of pixels of this center e(t)
def find_et():
	global et, etprev

	#keep track of et and e(t - 1), every new calculation resets it 



