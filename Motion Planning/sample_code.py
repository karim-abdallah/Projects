## Sample Function for Motion Planning and Trajectory Computation.
## Part of an Introduction to Robotics class at Columbia University given by Prof. Matei Ciocarlie.
## Uses RRT and PRM trajectory planning algorithms for motion planning in obstructed environments.
## Executes motion of a virtual render of Rethink Robotic's Baxter robot in ROS environment (http://wiki.ros.org/).
## All code written by Karim Abdallah, Connor Chamberlain and Jordan Loos. December 2015.


# Beginning of Motion Planning function. Takes joint values for start and goal position, and min/max joint values 
# determined by limitations of virtual Baxter robot. Returns joint values for intermediate positions, velocity 
# and acceleration values to be executed by virtual Baxter robot. 

def motion_plan(self, q_start, q_goal, q_min, q_max):

        # This simple example creates a trajectory with just the start and goal points
	v_list = []
	a_list = []
        q_list = []
        
        prm_start = []
        start_pos = -1
        end_pos = -1
        prm_goal = []
        q_list.append(q_start)
	use_RRT = True
	T =50
	timeout = 0

	#------------------------Main Menu -----------------------
	while True:
	
		print "Welcome to Baxter Motion planning"
		print "Choose 1 for RRT or 2 for PRM:"

		choice = raw_input('')
		choice = int(choice)
		if choice == 1:
			use_RRT = True
			break
		if choice == 2:
			use_RRT = False
			break
	
	# -----------------------Implementing PRM ---------------

	if use_RRT == False:	
		print "Beginning PRM Planning"
		current_time = 0
		start_time = current_time
		points = []
		points.append(q_start)
		q_connect = []
		
		while current_time - start_time <= T:
			q_rand = []
			

        		for j in range(0,7):
        			q_rand.append(random.uniform(q_min[j],q_max[j]))
        			
        		if self.is_state_valid(q_rand):
        			points.append(numpy.array(q_rand))
        		current_time = current_time+1
        		
        	points.append(q_goal)
		print 'Testing With: ' 
		print len(points) 
		print 'Valid Points'
        		
        	connections = numpy.identity(len(points))
        	d_matrix = 1000*numpy.identity(len(points))
        	visited = numpy.zeros((len(points),1))
     	        neighbors = numpy.zeros((len(points),1))
     	        dmap = numpy.zeros((len(points),1))
        	visited[0] = 1
        	visited[len(points)-1] = 1
        	        		
        	for a in range(len(points)):
        		
        		for b in range(a+1, len(points)):
        			dist = numpy.linalg.norm(points[b]-points[a])
        			
        			for j in range(1, int(dist//0.1)+1):
        		#print j
        				if self.is_state_valid(points[a]+(points[b]-points[a])*j/(10*dist)):

        					d_matrix[a][b]=	dist
        					d_matrix[b][a]= dist
        				else:
						d_matrix[a][b] = 1000
						d_matrix[b][a] = 1000
        					connections[b][a]=1
        					connections[a][b]=1
        					break
		


		for a in range(1, len(points)-1):
			#print numpy.linalg.norm(q_start-points[a])
			if numpy.linalg.norm(q_start-points[a]) == min(d_matrix[0]):
				prm_start.append(points[a])
				start_pos = a
				print 'prm_start found'
				print a
				
			#print numpy.linalg.norm(q_goal-points[a])
			if numpy.linalg.norm(q_goal-points[a]) == min(d_matrix[len(points)-1]):
				print 'prm_goal found'
				print a
				prm_goal.append(points[a])
				end_pos = a


		#print connections
		while sum(visited) != (len(visited)-1):

			visited[start_pos] = 1
			neighbors[start_pos] = 0
			for a in range(len(points)):
				if connections[start_pos][a] == 0 and visited[a] != 1:
					neighbors[a]=1
					if dmap[start_pos]+d_matrix[start_pos][a] < dmap[a] or dmap[a]== 0:	
						dmap[a]=dmap[start_pos]+d_matrix[start_pos][a]
			#print neighbors #neighbor works!
			if sum(neighbors) == 0:
				print neighbors
				print "PRM is not connected. Executing RRT motion planning"
				use_RRT = True
				break
			temp = []
			for a in range(len(points)):
				if neighbors[a]==1:
					temp.append(dmap[a])
			for a in range(len(points)):
				if dmap[a]==min(temp):
					neighbors[a]=0
					start_pos=a


			if sum(neighbors) == 0 and visited[end_pos] == 0:
				print "PRM is not connected.  Executing RRT motion planning."
				use_RRT = True
				break
		#print dmap
		path = []
		path.append(end_pos)
		cd = dmap[end_pos]	

	#------------------------- Run Dijkstra's ----------------------
		visited = numpy.zeros(len(points))
		
		while cd !=0 and use_RRT == False:
			temp = []
			visited[end_pos] = 1
			for a in range(len(points)):
				if connections[end_pos][a] == 0 and visited[a] != 1:
					temp.append(dmap[a]+d_matrix[a][end_pos])
			for a in range(len(points)):
				if dmap[a]+d_matrix[a][end_pos] == min(temp):
					path.append(a)
					end_pos=a
					cd=dmap[a]
					break
		if use_RRT == False:
			for a in range(len(path)-1):
				q_list.append(points[len(path)-1-a])
		
		q_list.append(q_goal) 
			
	
		
				
        	#print q_list		
				
			
	
        # -----------------------Implementing RRT------------------------
        # Step 1: Crude RRT

	v_list = []
	a_list = []
        #q_list = []
	#q_list.append(q_start)

	if use_RRT == True:
		q_list = []
		q_list.append(q_start)
        	print "Beginning Crude RRT Planning"
        	N = 0;
        	i = 0;
        	obstacle_collision = 0;
		timeout = 0;      
        	while True:
        		print N
        		#Check if Clear Path between q_i and q_goal
        		dist = numpy.linalg.norm(q_goal-q_list[i])
        		#print int(dist//0.1)
        		for j in range(1, int(dist//0.1)+1):
        			#print j
        			if self.is_state_valid(q_list[i]+(q_goal-q_list[i])*j/(10*dist)):
        				obstacle_collision = 0;
        			else:
        				obstacle_collision = 1;
        				break
        		if obstacle_collision == 0:
        			#print "Direct Path Available"
				#print q_goal
				#print q_goal[0]
				temp = numpy.array([q_goal[0],q_goal[1],q_goal[2],q_goal[3],q_goal[4],q_goal[5],q_goal[6]])
				#print temp
				q_list.append(temp)
				break
        		
        		
        		# Generating Random Tree
        		q_rand = []
			#print "Generating Random Node"
        		for j in range(0,7):
        			q_rand.append(random.uniform(q_min[j],q_max[j]))
        		
        		#print "Scaling Branch"
        		q_rand_scaled = 0.5*(q_rand - q_list[i])/numpy.linalg.norm(q_rand - q_list[i])
        		
        		#print "Checking if Obstacle"
        		
        		dist = numpy.linalg.norm(q_rand_scaled)
        		#print int(dist//0.1)
        		for j in range(1, int(dist//0.1)+1):
        			#print j
        			if self.is_state_valid(q_list[i]+(q_rand_scaled)*j/(10*dist)):
        				obstacle_collision = 0;
        			else:
        				obstacle_collision = 1;
        				break
        		if obstacle_collision == 0:
        			#if numpy.linalg.norm(q_goal-q_list[i]-q_rand_scaled)<numpy.linalg.norm(q_goal-q_list[i]):
        				#print "Direct Path Available"
				q_list.append(q_list[i]+q_rand_scaled)
				i = i+1
					#q_list.append(q_goal)
        				#break
			N = N+1
        		if N > 2000:
        			q_list = []
        			q_list.append(q_start)
        			print "Computation Timeout - No can do!"
				timeout = 1;
        			break
        			
        		#print "No Direct Path Available"

        		
        	#print q_list
        	
        	#break
        	
        	
        	print "Number of Crude Branches:"      	
        	print N
	# -----------------------Shortcutting------------------------
	
	#for p in range(3):	
	

	for y in range(1):
		k = 0;
		o = len(q_list)-2;
		if timeout == 0 and use_RRT == True:
			while True:
				
				dist = numpy.linalg.norm(q_goal - q_list[k])
        		
        			for j in range(1, int(dist//0.1)+1):
        			
        				if self.is_state_valid(q_list[k]+(q_goal-q_list[k])*j/(10*dist)):
        					obstacle_collision = 0;
        				else:
        					obstacle_collision = 1;
        					break
				if obstacle_collision == 0:
					q_list[k+1] = q_goal
					q_list = q_list[:k+2]
					break
					
			
				dist = numpy.linalg.norm(q_list[o] - q_list[k])
        		
        			for j in range(1, int(dist//0.1)+1):
        				
        				if self.is_state_valid(q_list[k]+(q_list[o]-q_list[k])*j/(10*dist)):
       						obstacle_collision = 0;
       					else:
       						obstacle_collision = 1;
       						break
				if obstacle_collision == 1:
					o = o-1
        			if obstacle_collision == 0:

					q_list[k+1] = q_list[o]
					k = k+1
					o = len(q_list)-2
	
		#print q_list
        
		print "Number of Nodes after Shortcut:"	
		print len(q_list)

	# -----------------------Re-sampling------------------------

		
		if timeout == 0 or use_RRT == False:
			temp = []
			temp.append(q_list[0])
			if numpy.linalg.norm(q_goal-q_list[0]) >= 1:			
				for a in range(len(q_list)-1):
					l = numpy.linalg.norm(q_list[a+1]-q_list[a])	
					print l
					n = int(l//0.5)
					print n
					difference = ((q_list[a+1])-(q_list[a]))

					for b in range(1,n+1):
						temp.append(q_list[a]+b*difference/n)
				q_list = temp


	#----------------------Trajectory Execution-------------------
	if timeout == 0 or use_RRT == False:
		N = len(q_list)
		print "N"
		print N
		A = numpy.zeros((4*N-4,4*N-4))
		#print A
		for n in range(N-1):
			A[2*n][4*n]=0#n**3
			A[2*n][4*n+1]=0#n**2
			A[2*n][4*n+2]=0#n
			A[2*n][4*n+3]=1
			A[2*n+1][4*n]=1#(n+1)**3
			A[2*n+1][4*n+1]=1#(n+1)**2
			A[2*n+1][4*n+2]=1#(n+1)
			A[2*n+1][4*n+3]=1
		for n in range(0,N-2):
			A[2*N-2+n][4*n]=3#3*(n+1)**2
			A[2*N-2+n][4*n+1]=2#2*(n+1)
			A[2*N-2+n][4*n+2]=1
			A[2*N-2+n][4*n+4]=0#-3*(n+1)**2
			A[2*N-2+n][4*n+5]=0#-2*(n+1)
			A[2*N-2+n][4*n+6]=-1#-1
			A[3*N-4+n][4*n]=6#*(n+1)
			A[3*N-4+n][4*n+1]=2
			A[3*N-4+n][4*n+4]=0#-6*(n+1)
			A[3*N-4+n][4*n+5]=-2
		A[4*N-6][2]=1
		A[4*N-5][4*N-8]=3#*(N-1)**2
		A[4*N-5][4*N-7]=2#*(N-1)
		A[4*N-5][4*N-6]=1
	#numpy.set_printoptions(precision=0,threshold='nan')
	#print A
	#print numpy.linalg.det(A)

		Q = numpy.zeros((4*N-4,7))
	#print Q
		for n in range(N-1):
			Q[2*n]=q_list[n]
			Q[2*n+1]=q_list[n+1]
	
	#print Q

	#X = numpy.linalg.solve(A,Q)
		X = numpy.dot(numpy.linalg.pinv(A),Q)
		#print X
		coef = []
		j = 4
		for i in range(4*len(q_list)-4):
			coef.append(X[i][j])
			
		a_list.append(2*X[1])
		
		v_list.append([0, 0, 0, 0, 0, 0, 0])
		for a in range(0,N-1):
			v_list.append(3*X[4*a] + 2*X[4*a+1] + X[4*a+2])
			a_list.append(6*X[4*a] + 2*X[4*a+1])

		
		self.plot_trajectory(len(q_list)-1, coef, 1)



	t = []
	for i in range(len(q_list)):
		t.append(1*i)

	#print X
	#print X[4*(N-1)-1]
	
	if timeout == 1:
		v_list,a_list,t = self.compute_simple_timing(q_list, 1)
	
	#v_list.append(3*X[4*N-8]*N**2 + 2*X[4*N-7]*N + X[4*N-6])
	
	#print v_list



        return q_list, v_list, a_list, t
        # ---------------------------------------------------------------
