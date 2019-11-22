#include "NS_Task_1_Sandbox.h"
using namespace std;

#define V 28
#define Max 1000
bool is_visited[10] = {1,1,0,0,0,0,0,0,0,0};    //9=6, 15=7, 10=8,16=9; //location of delivery points
bool mark =0;
bool rec_col = 0; 
bool called = 0;
struct vertex
{
	int x;
	int y;
	int z;
	vertex() {}
	vertex(int a, int b) :x(a), y(b) {}
}v[28],moves[12],curr_dir,req_dir,up[6];
vertex *dijkstra(int graph[][28], int , int );
void calculates(vertex *,vertex);
void u_turn(void);
void allign(void);
void alter_path(int);

int graph[V][V] = {
	//             {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27},
				   {0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//0
				   {4, 0, 8, 8,16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//1
				   {0, 8, 0, 0, 0, 8, 0,13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//2
				   {0, 8, 0, 0, 0, 0, 8, 0,13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//3
				   {0,16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0},//4
				   {0, 0, 8, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,16, 0, 0, 0, 0},//5
				   {0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,16, 0, 0, 0, 0},//6
				   {0, 0,13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0},//7
				   {0, 0, 0,13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0},//8
				   {0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//9
				   {0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//10
				   {0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0,14, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//11
				   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0,14, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//12
				   {0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0,14, 0, 0, 0, 0, 0, 0, 0, 0, 0,12, 0, 0, 0, 0, 0, 0},//13
				   {0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0,14, 0, 0, 0, 0, 0, 0, 0, 0, 0,12, 0, 0, 0, 0, 0},//14
				   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//15
				   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//16
				   {0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0},//17
				   {0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0},//18
				   {0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0},//19
				   {0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 6, 0, 0, 0, 0, 0},//20
				   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,12, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0},//21
				   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,12, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0},//22
				   {0, 0, 0, 0, 0,16,16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0},//23
				   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 4, 0, 0},//24
				   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 4, 4},//25
				   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0},//26
				   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0}	//27
};

void del_node(vertex cur_pos,int graph[][28])
{
	int node = moves[11].x;
	printf("deleting node between :{%d,%d}\n", moves[node].z, moves[node - 1].z);
	graph[moves[node].z][moves[node- 1].z] = 0;
}

void allign()
{
	printf("entering alligned with curr_dir:  (%d,%d)\n", curr_dir.x, curr_dir.y);
	if (curr_dir.x == 1 && curr_dir.y == 0)
	{
		printf("alligning left\n");
		left_turn_wls();
	}
	else if (curr_dir.x == -1 && curr_dir.y == 0)
	{
		printf("alligning right\n");
		right_turn_wls();
	}
	else if (curr_dir.x == 0 && curr_dir.y == 1)
	{
		printf("alligning straight\n");
	}

	printf("alligning stop\n");
	stop();
	_delay_ms(400);
	
	curr_dir.x = 0, curr_dir.y = 1;
	printf("leaving alligned with curr_dir:  (%d,%d)\n", curr_dir.x, curr_dir.y);
}

void al_d()
{
	if (curr_dir.x == 1 && curr_dir.y == 0)
	{
		printf("alligning right\n");
		right_turn_wls();
	}
	else if (curr_dir.x == -1 && curr_dir.y == 0)
	{
		printf("alligning left\n");
		left_turn_wls();
	}
	else if (curr_dir.x == 0 && curr_dir.y == -1)
	{
		printf("alligning straight\n");

	}
	curr_dir.x = 0, curr_dir.y = -1;
	
	printf("alligning stop\n");
	stop();
	_delay_ms(400);
	
	

}

 void follow(void)
{
	unsigned char Left, Mid, Right;           //declaring variables for white line sensor values
	int flag = 0, v = 150, t = 10;                    //variable to change velocity and delay time
	while (ADC_Conversion(4) > 15  && !flag)                             //run the loop until junction not detected
	{

		_delay_ms(1);
		Left = ADC_Conversion(1);
		Mid = ADC_Conversion(2);
		Right = ADC_Conversion(3);

		if (Left == 255 && Right == 255 && Mid == 255)               //junction condition
		{
			forward();                                        //move forward before stopping to allign wheels with track
			velocity(v, v);                                   //set velocity
			_delay_ms(480);                                         //exit condition
			flag = 1;
			
		}
		else if (Mid == 0 && Left > 200 && Right == 0)              //if central sensor is on the right side of the track
		{
			soft_left();                                         //move the bot left by a small amount
			velocity(v, v);
			_delay_ms(t);
		}
		else if (Mid == 0 && Left == 0 && Right > 200)          //if central sensor is on the left side of the track
		{
			soft_right();                                      //move the bot right by a small amount
			velocity(v, v);
			_delay_ms(t);
		}
		else if (Mid > 200 && Left > 200 && Right == 0)          //if central sensor and left sensor in line
		{
			soft_left();                                         //move bot left
			velocity(v, v);
			_delay_ms(t / 2);                                      //for vey small period of time
		}
		else if (Mid > 200 && Left == 0 && Right > 200)             //if central sensor and right sensor in line
		{
			soft_right();                                        //move bot right
			velocity(v, v);
			_delay_ms(t / 2);                                    //for vey small period of time
		}
		else if (Mid > 200 && Left == 0 && Right == 0)             //if central sensor is aligned on the track 
		{
			forward();                                          //move forward
			velocity(v, v);
			_delay_ms(t / 2);                                   //by a small amount
		}
		else if (Mid == 0 && Left == 0 && Right == 0)          //if none of the sensor is on the line
		{
			flag = 1;                                          //stop the bot
		}
		_delay_ms(t);                                         //refresh time of sensor data

	}
	printf("bahar...................................\n");
	stop(); //allow the bot to stabilize 

	_delay_ms(10);
	
	if (ADC_Conversion(4) < 20)
	{

		printf("andar...............................................\n");
		vertex *m;
		u_turn();
		follow();
		del_node(moves[11],graph);                          //redo size
		curr_dir.x = -1 * curr_dir.x, curr_dir.y = -1 * curr_dir.y;
		//for (int i = 0; i <= 11; i++)
			//printf("moves ------- (%d,%d,%d) at %d", moves[i].x, moves[i].y, moves[i].z,i);
		int source = moves[11].x;
		printf("source after deleting node  :%d\n", moves[source].z);
		alter_path(source);
		printf("called =1");
		called = 1;
		//m = dijkstra(graph, moves[source].z, 22);
		//printf(" source,dest :  (%d , %d)\n", source, m[11].y);
		//calculates(dijkstra(graph,source,m[11].y),curr_dir);
	}
    
}

 void alter_path(int source)
 {
	 vertex *m;
	 int src = moves[source].z;
	 
	 m = dijkstra(graph, moves[source].z, 22);
	
	 printf("source when calling dijkstras from alter path ::   %d\n", src);
	 printf("destination when calling dijkstras from alter path ::   %d\n", moves[11].y);
	 printf("direction when calling dijkstras from alter path ::   %d,%d\n", curr_dir.x, curr_dir.y);

	 
	 calculates(dijkstra(graph, src,moves[11].y), curr_dir);
	 printf("leaving alter path....................\n");

 }


/*
*
* Function Name: forward_wls
* Input: node
* Output: void
* Logic: Uses white line sensors to go forward by the number of nodes specified
* Example Call: forward_wls(2); //Goes forward by two nodes
*s
*/ 
void forward_wls(unsigned char node)
{
	unsigned char count = 0;            //counter for number of junctions
	while (count < node)                //run the loop until given no. of junctions are not crossed
	{
		forward();                     //move forward while checking values at small intervals
		if (ADC_Conversion(1) > 200 && ADC_Conversion(2) > 200 && ADC_Conversion(3) > 200)  //junction condition
		{
			count++;                  //increment counter at every junction
			forward();                //move forward inorder to avoid counting the same junction
			velocity(200, 200);
			_delay_ms(370);
		}
		_delay_ms(10);   //refresh rate of sensor
	}
	stop();      //allow the bot to stabilize
	_delay_ms(20);

}
/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls(); //Turns right until black line is encountered
*
*/
void left_turn_wls(void)
{
	
	left();           //turn left to avoid reading current state data
	_delay_ms(50);
	while (!(ADC_Conversion(1) >200 && ADC_Conversion(2) > 200 && ADC_Conversion(3) == 0))  //loop until central sensor is not on track
	{
		
		left();
		velocity(50, 50);    //turn left slowly to avoid overshoot
		_delay_ms(30);       //refresh rate of sensor
	}
	stop();            //allow the bot to stabilize
	_delay_ms(20);
}
/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls(); //Turns right until black line is encountered
*/
void right_turn_wls(void)
{
	right();                          //turn left to avoid reading current state data
	_delay_ms(50);                   // 1 is left sensor
	while (!(ADC_Conversion(1) == 0 && ADC_Conversion(2) > 200 && ADC_Conversion(3) > 200))  //loop until central sensor is not on track
	{


		right();
		velocity(50, 50);         //turn left slowly to avoid overshoot
		_delay_ms(30);            //refresh rate of sensor
	}
	stop();             //allow the bot to stabilize
	_delay_ms(20);
	
}
void u_turn(void)
{
	left();
	_delay_ms(800);
	left_turn_wls();
}
vertex calc_dir(vertex pos, vertex next)
{
	vertex dir;
	if (next.x - pos.x)
		dir.x = (next.x - pos.x) / abs(next.x - pos.x);
	else
		dir.x = (next.x - pos.x);

	if (next.y - pos.y)
		dir.y = (next.y - pos.y) / abs(next.y - pos.y);
	else
	 	dir.y = (next.y - pos.y);

	return dir;

}

void turn(vertex req_dir, vertex curr_dir)
{
	
	printf("required path (%d,%d)\n",req_dir.x,req_dir.y);
	
	/*if (moves[moves[11].x].z == 6 && req_dir.x == 1 && req_dir.y == 0)
	{
		follow();
		right_turn_wls();
	}

	else if (moves[moves[11].x].z == 5 && req_dir.x == -1 && req_dir.y == 0)
	{
		left_turn_wls();
				follow();
				left_turn_wls();
	}*/

	if (req_dir.x == (-1 * curr_dir.x ) && req_dir.y == (-1 * curr_dir.y))
	{
		printf("turning back\n");
		u_turn();
		follow();
	}
	else if (curr_dir.x == req_dir.x && curr_dir.y == req_dir.y)
	{
		follow();
		printf("same direction straight\n");
	}
	
	else if (curr_dir.x == 0 && curr_dir.y == 1)           //north
	{
		

		if (req_dir.x == 1 && (req_dir.y == 0 || req_dir.y == 1))
		{
			right_turn_wls();
			follow();
			printf("right  2\n");
		}


		else if (req_dir.x == -1 && (req_dir.y == 1 || req_dir.y == 0))
		{
			left_turn_wls();
			follow();
			printf("diagonal left  4\n");
		}



	}
	else if (curr_dir.x == -1 && curr_dir.y == 0)          //west
	{
		if ((req_dir.x == 0 || req_dir.x == -1) && req_dir.y == 1)
		{
			right_turn_wls();
			follow();
			printf("right  11\n");
		}

		else if ((req_dir.x == 0 || req_dir.x == -1) && req_dir.y == -1)
		{
			left_turn_wls();
			follow();
			printf("left   12\n");
		}

	}

	else if (curr_dir.x == 1 && curr_dir.y == 0)         //east
	{
		if ((req_dir.x == 0 || req_dir.x == 1) && req_dir.y == 1)
		{
			left_turn_wls();
			follow();
			printf("left   21\n");
		}

		else if ((req_dir.x == 0 || req_dir.x == 1) && req_dir.y == -1)
		{
			right_turn_wls();
			follow();
			printf("right  23\n");
		}



	}

	else if (curr_dir.x == 0 && curr_dir.y == -1)          //south
	{
		if (req_dir.x == 1 && req_dir.y == 0 )
		{
			left_turn_wls();
			follow();
			printf("left 31\n");
		}
		else if (req_dir.x == -1 && req_dir.y == 0)
		{
			right_turn_wls();
			follow();
			printf("right    33\n");
		}

		else if (req_dir.x == -1 && req_dir.y == -1 || req_dir.x == 1 && req_dir.y == -1)
		{
			follow();
			printf("move straight in diagonal 34\n");
		}


	}
	else if (curr_dir.x == 1 && curr_dir.y == 1)          //1st quadrant
	{
		if (req_dir.x == 1 && req_dir.y == 0)
		{
			right_turn_wls();
			follow();
			printf("right  41\n");
		}
		else if (req_dir.x == 0 && req_dir.y == 1)
		{
			
			follow();
			printf("straight diagonally  42\n");
		} 
	} 

	else if (curr_dir.x == -1 && curr_dir.y == 1)          //2nd quadrant
	{
		if (req_dir.x == -1 && req_dir.y == 0)
		{
			left_turn_wls();
			follow();
			printf("left  51\n");
		}
		else if (req_dir.x == 0 && req_dir.y == 1)
		{
			follow();
			printf("straight diagonally  52\n");
		}
	}

	else if (curr_dir.x == -1 && curr_dir.y == -1)          //3rd quadrant
	{
		if (req_dir.x == -1 && req_dir.y == 0)
		{
			right_turn_wls();
			follow();
			printf("right  61\n");
		}
		else if (req_dir.x == 0 && req_dir.y == -1)
		{
			left_turn_wls();
			follow();
			printf("left  62\n");
		}
		
	}
	else if (curr_dir.x == 1 && curr_dir.y == -1)          //4th quadrant
	{
		if (req_dir.x == 1 && req_dir.y == 0)
		{
			left_turn_wls();
			follow();
			printf("left  71\n");
		}
		else if (req_dir.x == 0 && req_dir.y == -1)
		{
			right_turn_wls();
			follow();
			printf("right  72\n");
		}
		
	}


}

int minDistance(int dist[], bool sptSet[])
{

	// Initialize min value 
	int min = Max, min_index;

	for (int v = 0; v < V; v++)
		if (sptSet[v] == false && dist[v] <= min)
			min = dist[v], min_index = v;

	return min_index;
}


// Function to print shortest 
// path from source to j 
// using parent array 



// Funtion that implements Dijkstra's 
// single source shortest path 
// algorithm for a graph represented 
// using adjacency matrix representation 
vertex *dijkstra(int graph[V][V], int src, int d)            
{
	
	int dist[V];
	//array to store shortes nodes from obstacl
	// sptSet[i] will true if vertex 
	// i is included / in shortest 
	// path tree or shortest distance 
	// from src to i is finalized 
	bool sptSet[V];

	// Parent array to store 
	// shortest path tree 
	int parent[V];
	
	// Initialize all distances as 
	// INFINITE and stpSet[] as false 
	for (int i = 0; i < V; i++)
	{
		parent[0] = -1;
		dist[i] = Max;
		sptSet[i] = false;
	}
	
	// Distance of source vertex 
	// from itself is always 0 
	dist[src] = 0;

	// Find shortest path 
	// for all vertices 
	for (int count = 0; count < V - 1; count++)
	{
		
		// Pick the minimum distance 
		// vertex from the set of 
		// vertices not yet processed. 
		// u is always equal to src 
		// in first iteration. 
		int u = minDistance(dist, sptSet);

		// Mark the picked vertex 
		// as processed 
		sptSet[u] = true;

		// Update dist value of the 
		// adjacent vertices of the 
		// picked vertex. 
		
		for (int v = 0; v < V; v++)

			// Update dist[v] only if is 
			// not in sptSet, there is 
			// an edge from u to v, and 
			// total weight of path from 
			// src to v through u is smaller 
			// than current value of 
			// dist[v] 
			if (!sptSet[v] && graph[u][v] &&
				dist[u] + graph[u][v] < dist[v])
			{
				parent[v] = u;
				dist[v] = dist[u] + graph[u][v];
			}
	}

	
	int ind_r = 10;  //loc of destinaton red
	int ind_g = 15;  //loc of green dest
	if (mark==0)    // loc of top nodes  mark=0 means searching for top nodes
					
	{
		int minimum = INT_MAX;
		//int minimum = dist[17];
		static int index = 17;
		for (int c = 17; c < 23; c++)
		{
			//printf("distance array  %d     \n", dist[c]);
			if (dist[c] < minimum && !is_visited[c-17])
			{
				minimum = dist[c];
				index = c;
			}
		}
		printf("index in function :%d", index);
		moves[11].y = index;
	}
	else if (mark == 1 && rec_col == 0)  //mark=1 means searching for bottom nodes
	{
		//9 15
		if ((dist[9] < dist[15]) && !is_visited[6])
		{
			ind_g = 9;
		}
		moves[11].y = ind_g;     //green
	}
	else if (mark == 1 && rec_col == 1 )
	{
		//10 16
		if (dist[16] < dist[10]  && !is_visited[9])
		{
			ind_r = 16;
		}
		moves[11].y = ind_r;
	}
	printf("index of upper nodes from current pos:  %d\n", moves[11].y);
		
	
	// print the constructed 
	// distance array 

	//printf("source in disjkstra :  %d", src);
	//printf("destination in disjkstra :  %d", moves[11].y);
	printf("\n(%d,%d) -> (%d,%d) \t\t %d\t\t(%d,%d) \n ", v[src].x, v[src].y, v[d].x, v[d].y, dist[d], v[src].x, v[src].y);
	int i = 0;
	moves[0].z=d;
	
	printf("move 0 %d,%d\n", moves[0].x, moves[0].y);
	while (parent[d] >= 0)
	{
		printf("parent %d\n", parent[d]);
		moves[i].x = v[d].x, moves[i].y = v[d].y;
		i++;
		moves[i].z = parent[d];
		d = parent[d];
	}
	moves[i].x = v[src].x, moves[i].y = v[src].y, moves[i].z=src;
	moves[11].x = i;    //current position
	//printf("\nval of i%d\n", moves[11].x);
	return moves;
}


/*
*
* Function Name: follow
* Input: void
* Output: void
* Logic: using the values of white line sensor to keep the bot on the specified track
* Example Call: follow(); //follows curve until junction point is detected
*/
/*
*
* Function Name: Square
* Input: void
* Output: void
* Logic: Use this function to make the robot trace a square path on the arena
* Example Call: Square();
*/
void Square(void)
{

}
void calculates(vertex moves[ ],vertex dir)          //curr_pos pass kiya tha
{
	
	int size = moves[11].x;
	printf("current position in (x,y,z):   (%d  ,%d  ,%d)\n", moves[size].x, moves[size].y, moves[size].z);
	turn(calc_dir(moves[size], moves[size - 1]), dir);
	curr_dir = calc_dir(moves[size], moves[size - 1]);
	size--;
	moves[11].x--;
	while (size > 0)
	{
		printf("current position in (x,y, index):   (%d  , %d ) ,%d\n", moves[size].x, moves[size].y, moves[size].z);
		turn(calc_dir(moves[size], moves[size - 1]), curr_dir);
		if(called)
		{
			printf("breaking\n");
			break;
		}

		printf("size %d\n", size);
		curr_dir = calc_dir(moves[size], moves[size - 1]);
		moves[11].x--;    //stores curr_position indices
		size--;
	}
	printf("current position in (x,y, index):   (%d  , %d ) ,%d\n", moves[size].x, moves[size].y, moves[size].z);

	//curr_dir = calc_dir(moves[size+1], moves[size]);
	printf("leaving calculates  (%d , %d)\n", curr_dir.x, curr_dir.y);
	called = 0;
}

int color()
{
	int r_pc, g_pc, b_pc, filter_clear_pc;
	filter_red();
	r_pc = color_sensor_pulse_count;
	filter_clear();
	filter_green();
	g_pc = color_sensor_pulse_count;
	filter_clear();
	_delay_ms(100);
	if (r_pc > g_pc)
	{
		return 2;
	}
	else if (g_pc > r_pc)
	{
		return 3;
	}
	else
		return 0;
}
/*
*
* Function Name: Task_1_1
* Input: void
* Output: void
* Logic: Use this function to encapsulate your Task 1.1 logic
* Example Call: Task_1_1();
*/
void Task_1_1(void)
{
	
}
/*
*
* Function Name: Task_1_2
* Input: void
* Output: void
* Logic: Use this function to encapsulate your Task 1.2 logic
* Example Call: Task_1_2();
*/
void Task_1_2(void)
{
	follow();
	stop();
	_delay_ms(100);
	int s, det,det1;

	scanf_s("%d", &s);   //source
	scanf_s("%d", &det);  //destination
	v[0].x = 0, v[0].y = 0;
	v[1].x = 0, v[1].y = 1;
	v[2].x = 1, v[2].y = 1;
	v[3].x = -1, v[3].y = 1; 
	v[4].x = 0, v[4].y = 4;
	v[5].x = 1, v[5].y = 0;
	v[6].x = -1, v[6].y = 0;
	v[7].x = 2, v[7].y = 3;
	v[8].x = -2, v[8].y = 3;
	v[9].x = 2, v[9].y = 0;
	v[10].x = -2, v[10].y = 0;
	v[11].x = 3, v[11].y = 0;
	v[12].x = -3, v[12].y = 0;
	v[13].x = 3, v[13].y = 3;
	v[14].x = -3, v[14].y = 3;
	v[15].x = 3, v[15].y = -1;
	v[16].x = -3, v[16].y = -1;
	v[17].x = 1, v[17].y = 4;
	v[18].x = -1, v[18].y = 4;
	v[19].x = 2, v[19].y = 4;
	v[20].x = -2, v[20].y = 4;
	v[21].x = 3, v[21].y = 4;
	v[22].x = -3, v[22].y = 4;
	v[23].x = 0, v[23].y = -1;
	v[24].x = 0, v[24].y = -2;
	v[25].x = 0, v[25].y = -3;
	v[26].x = 1, v[26].y = -3;
	v[27].x = -1, v[27].y = -3;

	vertex go(0, 1);
	calculates(dijkstra(graph, s, det),go);
	printf("called dijkstras once calling allign \n");
	//scanf_s("%d", &det1);  //destination
	//printf("direction after 1st call (%d,  %d)\n", curr_dir.x, curr_dir.y);
	//printf("position after 1st call (%d,  %d , %d)\n",moves[moves[11].x].x, moves[moves[11].x].y, moves[moves[11].x].z);
	//calculates(dijkstra(graph, det, det1), curr_dir);
	for (int i = 0; i < 4; i++)
	{
		printf("current direction  : (%d,%d)\n", curr_dir.x, curr_dir.y);
		allign();
		//int col = color();
		//printf("%d",col);
		//printf("current dir after alligned is called :: (%d  ,  %d)", curr_dir.x, curr_dir.y);
		//if (!color())
		//{
			printf("found colour \n\n\n\n\n\n");
			int col = color();
			if (col == 3)      //colour red
			{
				rec_col = 0;
				//green zone
				pick();
				is_visited[moves[11].y - 17] = 1;
				mark = 1;
				printf("calling alter path at psotion: and colour green  %d\n", moves[moves[11].x].z);
				alter_path(moves[11].x);
				//dijakstra call, calculating destination for zone D2
				// calculate call for shortest path
				//allign(curr_dir);
				al_d();
				place();
				printf("placing green at :%d\n", moves[11].y);
				if (moves[11].y == 9)
					is_visited[6] = 1;
				else if (moves[11].y == 15)
					is_visited[7] = 1;
				mark = 0;
			}
			else if (col == 2)  //colour red
			{
				rec_col = 1;
				//red zone
				pick();
				is_visited[moves[11].y - 17] = 1;
				mark = 1;
				printf("calling alter path at position:and colour red  %d\n", moves[moves[11].x].z);
				alter_path(moves[11].x);
				//dijakstra call, calculating destination for zone D1
				// calculate call for shortest path
				//allign(curr_dir);
				al_d();
				place();
				printf("placing red at :%d\n", moves[11].y);
				if (moves[11].y == 10)
					is_visited[8] = 1;
				else if (moves[11].y == 16)
					is_visited[9] = 1;

				mark = 0;
			}
		
		//else  pass(make code for searching new nut)
		for (int i = 0; i < 10; i++)
			printf("visted val at %d       is :  %d   \n ", i, is_visited[i]);
		alter_path(moves[11].x);
	} 
}