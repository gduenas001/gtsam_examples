
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv



# initialize figure
fig = plt.figure()
ax= plt.subplot(111, projection='3d')

# add estimated positions to the plot
estimated_positions_x= []
estimated_positions_y= []
estimated_positions_z= []
with open('../results/estimated_positions.csv', 'rb') as csvfile:
     estimated_positions = csv.reader(csvfile, delimiter=',')
     for position in estimated_positions:
          try:
               estimated_positions_x.append(float(position[0]))
               estimated_positions_y.append(float(position[1]))
               estimated_positions_z.append(float(position[2]))
          except:
               pass

# add estimated positions to the plot
true_positions_x= []
true_positions_y= []
true_positions_z= []
with open('../results/true_positions.csv', 'rb') as csvfile:
     true_positions = csv.reader(csvfile, delimiter=',')
     for position in true_positions:
          try:
               true_positions_x.append( float(position[0]) )
               true_positions_y.append( float(position[1]) )
               true_positions_z.append( float(position[2]) )
          except:
               pass
		
# add landmarks to the plot
landmarks_x= []
landmarks_y= []
landmarks_z= []
with open('../results/landmarks.csv', 'rb') as csvfile:
     true_positions = csv.reader(csvfile, delimiter=',')
     for position in true_positions:
          landmarks_x.append( float(position[0]) )
          landmarks_y.append( float(position[1]) )
          landmarks_z.append( float(position[2]) )


plt.plot(estimated_positions_x, estimated_positions_y, estimated_positions_z, color= 'b', linestyle='-', marker='o')
plt.plot(true_positions_x, true_positions_y, true_positions_z, color= 'r', linestyle='-')
ax.scatter3D(landmarks_x, landmarks_y, landmarks_z, color= 'g', marker='^', s=100)

x_max = max(max(estimated_positions_x) , max(landmarks_x)) + 1
y_max = max(max(estimated_positions_y) , max(landmarks_y)) + 1
z_max = max(max(estimated_positions_z) , max(landmarks_z)) + 1
x_min = min(min(estimated_positions_x) , min(landmarks_x)) - 1
y_min = min(min(estimated_positions_y) , min(landmarks_y)) - 1
z_min = min(min(estimated_positions_z) , min(landmarks_z)) - 1

max_range = np.array([x_max - x_min, y_max - y_min, z_max - z_min]).max() / 2.0

mid_x = ( x_max + x_min ) * 0.5
mid_y = ( y_max + y_min ) * 0.5
mid_z = ( z_max + z_min ) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)


# add axis
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')


plt.show(block=False)
plt.pause(10)
plt.close()

# plt.show()
print "figure closed"













# x = np.linspace(0, 10, 500)
# y = np.sin(x)

# fig, ax = plt.subplots()

# # Using set_dashes() to modify dashing of an existing line
# line1, = ax.plot(x, y, label='Using set_dashes()')
# line1.set_dashes([2, 2, 10, 2])  # 2pt line, 2pt break, 10pt line, 2pt break

# # Using plot(..., dashes=...) to set the dashing when creating a line
# line2, = ax.plot(x, y - 0.2, dashes=[6, 2], label='Using the dashes parameter')

# ax.legend()
# plt.show()