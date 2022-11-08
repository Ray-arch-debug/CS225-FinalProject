# CS225-FinalProject
Final Project for CS-225

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



# TEAM CONTRACT 

## Communication
1. **Team Meetings** 
The meetings will be hosted twice a week, once in-person in Daniel's Hall and once on discord on the following friday just to catch up and make sure we get through what we needed to get through. Aakash will take in person meeting notes and the discord calls will also be sumarized by Aakash. 
2. **Assistance** 
Our team is using a discord server in order to make these conversations easier, furthermore meeting in person is also being enforced. The discord server is our primary source of conversation but we will also be using Snapchat and text to hold private in group conversations as needed.
3. **Respect** 
Our team is very good at giving everyone a chance to present their ideas and is supportive of other people's ideas, the coolection of notes and the constant emphasis in teamwork will be reinforced by the notetaker to further develop this already respectful environment. 

## Collaboration

4. **Work Distribution** 
The work distribution will be based on the amount of work finished and our deliverables will be split in parts where each member will focus on different aspects of the project to make sure it is delivered on time. We do not believe in boundaries or our work system, we will work on other parts of project together if we finish our parts. We have split the project into multiple parts, research, data analysis, algorithms and post analysis and we will use these to split our work in multiple parts and work across each as a team. 
5. **Time Commitment** 
Our group has decided that it is crucial that we each spend about 4 hours outside of our regular meeting times in order to make progress on the project. We have discussed the possibility of time commitement and made up a convinenient time to meet up. However, the group work is worked on remotely while, we will use the discord server to discuss the problems which arise. We have made the server to have serveral functionalities which make our lives easier in term of keeping our communications clear which should improve efficienct and help us resolve time conflicts. 

6. **Conflict Resolution** 
Our group is very tolerative of understanding other parties problems, we will enforce and expect everyone to present the team with viable reasons for being late and this will be handled by the note-taker (manager) of the group. We will also be patience but very strict about our deadlines within the group, however, we will also make concessions and work together when facing difficultites regarding the project portions. We also realize that we have a official mentor for this project who we will reach out to when we find ourselves at odds with each other.


## Signatures
Brendan Biernacki (bab8)

Shaurya Singh (shaurya8)

Alexander Wang (aw26)

Aakash Kumar (akuma29)


---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



# PROJECT PROPOSAL 

## Leading Question: What is the best alternative fuels vehicle to use to travel from one location to another in the USA? 

The use of fossil fuel vehicles poses a problem for climate change and meeting carbon emissions goals. People would like to know how they can travel whilst minimizing their emissions, and to do so they invest in alternative fuel vehicles (electric cars, lng powered cars, etc). However, the lack of refueling infrastructure in a given location poses a problem for the user. It is entirely possible their alternative fuel vehicle simply doesnt have the road infracture accompanying it available to be used for reaching one location to another. For example, a bio-diesel powered vehicle would find it impossible to go from Las Vegas to Denver as there are no bio-diesel refueling stations between the two cities. Likewise, a conscious consumer should invest in the alternative fuel vehicle that has the most infrastructure support in their area. By using MST algorithms such as Prims or Kruskals, we can find the maximum range of a alternative fuel vehicle given a starting point and the maximum distance it can go on a full tank as input by the user. This can also show us the infrastructure coverage. A way to compare the alternative fuels with current petrol vehicle coverage can be done by assuming that the coverge of petrol is infinite and seeing which alternative fuel comes close enough to matching that. By infinite we mean that one is never hampered by lack of petrol fueling infrastructure hence the coverage is infinite as there are enough petrol pumps to never have to worry about running out of gas. Also, by using Dijkstra's and other path finding algorithms we can find the shortest possible route a vehicle can travel given user inputs on locations, without having to worry about running out of fuel in between (as we check for fueling stations in between). This tool can then be used to recommend which route to take for a user, and also give insights as to which fuel type car the user should buy (eg: lng vs bio-diesel). There are also policy making implications, as the lack of paths between locations could be used to find out weak spots in the grid so we know that infrastructure should be constructed there to create paths. There are a lot of features you could add on, however we will focus mainly on reachability and optimal routes as explained above. The dataset we will use is the "alternative fueling station locator" provided by the US department of energy. In broad terms, we can use this dataset, which provide the coordinates and types of different fueling stations to find out the distance between then by extention find reachability and optimum routes. Specifications for data below. 

## Dataset Acquisition:

We will be using the "alt_fuels_stations" dataset: https://afdc.energy.gov/data_download/, and an example of the application of the dataset for context can be found here: https://afdc.energy.gov/stations/#/find/nearest.  The dataset is a csv, and we will use subsets of it as variables to drive our algorithms. 

## Data Format: 

The datasets is in a .csv format. It has a size of 21.1 MB.  

The input formats is as follows: 
Fuel Type Code, Station Name, Street Address, Intersection Directions, City, State, ZIP, Plus4,	Station Phone, Status Code, Expected Date, Groups With Access Code... and many more, however we will only be using a subset of this data. Currenty we only need: Fuel Type Code, City, Stae, Latitude, Longitude. There may be more added should the need arise in the future, but currently this should suffice. 

Fuel type will help us differentiate between alternative fuel vehciles, and latitude and longitude will help us calculate distances for routes and reachability/ range, and finally the City/State value will help us generate a readable output. 36.1716° N, 115.1391° W is not as easy to read as Las Vegas for example, and we will need to convert between user inputs for locations and then coordinates. 

The range of their vehicle (for reachability calculations) is more of a parameter than a real piece of data that needs to be stored somewhere as it can be discarded after runtime, so we wont be storing it in the dataset. 


## Data Correction:

Any object missing any of the above subset values will not be used as all of them are vital for the functionality of the program. We will use the commas to extract different values (as usual with csv), once we have checked for missing values, and then create objects to use as nodes on the map, filled with their respective elements from the dataset. 

## Data Storage:


We could use an adjacency list consisting of a map from node pointers to min-heaps, sorted by the distance from the vertex. This has the downfall that it is disposable - we must reconstruct the graph each time, costing $O(nd)$, where $d$ is the largest degree of a vertex, since constructing each min-heap should take $O(d)$ time if we use the algorithm discussed in lecture and lab_heap. We will only need to access each edge once in any BST, DST, Prim's, or Kruskal's, so this should work. However, accessing each edge in order will take $O(d)$ time, which is not constant.

Using Prim's on this could potentially be more efficient than the algorithm presented in lecture, under the assumption that the edges are evenly distributed among the vertices. That is, $d=O(m/n)$.

1. Construct the graph as described above using a priority queue to store the minimum incident edge to a vertex. By the partition rule, this edge must be in a MST. Doing this for every vertex should identify at least $n/2$ edges that belong in the MST.

2. For each component of the graph, store all the unused minimum incident edges for the vertices in that component in another min-heap. That should identify a new edge that is part of a MST. Doing this for all components reduces the number of components by a factor of $2$. Identifying the minimum outgoing edge from each vertex takes constant time. (I'm not sure about this, because these edges aren't guaranteed to join this component with a different one. But my runtime seems oddly low so I probably have a bug here.)

3. Do this until the MST is connected. If I'm right about this, each step should take $O(\frac{n}{2^i}i)$ time where $i$ is the current step number. Summing for all $1\le i\le \log n$ gives a total runtime of $O(n)$ (!). Adding in the time it took to construct the graph, we get the runtime is $O(n+nd)=O(m)$. Again, I my algorithm is probably incorrect, but there is potential to debug it and it could still be faster because there is so much wiggle room here.




Tips - Graph, with nodes filled with data highlighted above, and have adjaceny matrix for the nearest direct ones (without stops in between). Calculated using latitude and longitude. The weighted edges can be used with BFS and DFS to find the best route and reach. The question has to do more with what kind of graph implementation we will use. adjacency matrix or edge list


## Algorithm

We will be using a few different algorithms. Our estimate goal on the Big O efficiency in terms of runtime is the same as covered in class. In particular, we will use Kruskal's and Prim's algorithms to measure the reachibility to other cities based on the type of fuel. These have a runtime of $O(n + m lon(n))$ and $O(nlog(n) + mlog(n))$, respectively. Meanwhile, to find the shortest distance, we will use Dijkstra's, which has a runtime of $O(nlog(n))$. Lastly, for graph traversal, we will use BFS and DFS. 

## Timeline

- Finish the project proposal. (11/4)

- Write the necessary test cases and begin implementation of the graph and algorithms. We will not be passing all of our test cases at this point, but we should have a solid idea of what we need to debug in the next week. Until we pass all of our tests, we will not touch our dataset unless it is to test if we're reading it in correctly. (11/11)

- Finish debugging our algorithms. Our tests should include some way to make sure our big-O runtime is correct. Once we are confident in our implementations, we will have our mentor meeting to discuss if anything could be improved and steps after performing our algorithms on the dataset. We should have some way of verifying the algorithm performs reasonably on our test, but this might not be a formal test case if we cannot retrieve much information about our algorithm performance at this point. (11/18)

- Think about how to present our findings. Ideally, we could display our graph overlayed on a map of the United States, and color the visited edges and nodes in our MST or BFS/DFS algorithms in some color. Record the video and finish writing the final results. Wrap up anything else. (12/2)

We are giving ourselves a week buffer because we see us procrastinating on these checkpoints, like we did for this project proposal.
