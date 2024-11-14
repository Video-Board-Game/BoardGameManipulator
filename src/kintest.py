import numpy as np
# import sympy as sp
import fk
# import vk
import ik
import matplotlib.pyplot as plt
import time

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

cycles = np.array([])
execution_times = np.array([])
execution_hz = np.array([])
goal_pos=np.array([])
goal_angs=np.array([])
educated_guess_angs=np.array([])
educated_guess_pos=np.array([])
educated_guess_error=np.array([])
result_angs=np.array([])
result_pos=np.array([])


for n in range(200000):
    angs=np.random.uniform(-np.pi/3,np.pi/3,(3,1))
    pos=fk.arm2fk(angs[0],angs[1],angs[2])[0:3,3]
    goal_pos=np.append(goal_pos,pos)
    goal_angs=np.append(goal_angs,angs)
    theta0_d = np.arctan2(pos[0],-pos[2])*1.25
    theta2_d = -np.arctan2(pos[0],-pos[2])/2
    theta1_d = np.arcsin(pos[1]/(.18+.18*np.sin(theta2_d))) * np.sign(theta0_d)
    educated_guess_angs=np.append(educated_guess_angs,[theta0_d,theta1_d,theta2_d])
    educated_guess_pos=np.append(educated_guess_pos,fk.arm2fk(theta0_d,theta1_d,theta2_d)[0:3,3])
    educated_guess_error=np.append(educated_guess_error,np.linalg.norm(pos-educated_guess_pos[n*3:(n+1)*3]))
    start_time = time.time_ns()
    ikangs=ik.arm2ik(pos[0],pos[1],pos[2])
    execution_time =time.time_ns()-start_time
    execution_time*=1e-9
    if(execution_time<0.001):
        execution_time=0.001
    result_angs=np.append(result_angs,ikangs[0:3])
    result_pos=np.append(result_pos,fk.arm2fk(ikangs[0],ikangs[1],ikangs[2])[0:3,3])
    cycles=np.append(cycles,ikangs[3])
    execution_times=np.append(execution_times,execution_time)
    execution_hz=np.append(execution_hz,1/(execution_time))


educated_ang_error=np.abs(wrap_to_pi(goal_angs-educated_guess_angs))

print("Mean cycles: ",np.mean(cycles))
print("Max cycles: ",np.max(cycles))
print("Min cycles: ",np.min(cycles))
print("Std cycles: ",np.std(cycles))
print("Median cycles: ",np.median(cycles))
print()

print("Mean execution time (s): ", np.mean(execution_times))
print("Max execution time (s): ", np.max(execution_times))
print("Min execution time (s): ", np.min(execution_times))
print("Std execution time (s): ", np.std(execution_times))
print("Median execution time (s): ", np.median(execution_times))
print()

print("Mean educated guess error: ", np.mean(educated_guess_error))
print("Max educated guess error: ", np.max(educated_guess_error))
print("Min educated guess error: ", np.min(educated_guess_error))
print("Std educated guess error: ", np.std(educated_guess_error))
print("Median educated guess error: ", np.median(educated_guess_error))
print()

print("Mean execution hz: ", np.mean(execution_hz))
print("Max execution hz: ", np.max(execution_hz))
print("Min execution hz: ", np.min(execution_hz))
print("Std execution hz: ", np.std(execution_hz))
print("Median execution hz: ", np.median(execution_hz))
print()


plt.figure()
# Set the x-axis to log scale
plt.xscale('log')
hist, bins, _ = plt.hist(execution_hz, bins=10,log=True)
plt.clf()
logbins = np.logspace(np.log10(bins[0]),np.log10(bins[-1]),len(bins))
plt.hist(execution_hz, bins=logbins,log=True)
plt.xscale('log')
plt.title('Execution Times')
plt.xlabel('Execution Times (Hz)')
plt.show()