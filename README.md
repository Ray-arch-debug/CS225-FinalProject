<div align = "center">
  
  <img src = "forefront.png">
  
  <p align = "center">
    <h1>AUTOMOBILES AND ALTERNATE FUELS</h1>
    <p><h3>Created by Aakash Kumar, Brendan Biernacki, Shaurya Singh, and Alex Wang</h3></p>
  </p>
  
  <p align = "center">
    <a href = "https://github.com/Ray-arch-debug/CS225-FinalProject/blob/main/documents/Team%20Proposal">Project Proposal</a>
    |
    <a href = "https://youtu.be/eFeh2Rhlu20">Presentation Video</a>
    |
    <a href = "https://github.com/Ray-arch-debug/CS225-FinalProject/tree/main/data">Data</a>
    |
    <a href = "https://github.com/Ray-arch-debug/CS225-FinalProject/blob/main/documents/Project%20Name_%20Automobiles%20and%20Alternate%20Fuel%20Pumps.pdf"> Project Write Up</a>
    |
    <a href = "https://imsa0-my.sharepoint.com/:p:/g/personal/akumar2_imsa_edu/EboEVjgbFLpHgWCcQWVHgEIBMn095ArRdHpLF4jKkgxYkg?rtime=TvSqwL_c2kg"> Project Slides </a> 
<!--     <a href = "https://docs.google.com/document/d/16Ol95jGr3P_oHxa4LqEG1_2wpmvbBIXkynpoy6MEi_M/edit?usp=sharing">Project Structure</a> -->
  </p>
</div>

<details>
  <summary><h2>Table of Contents</h2></summary>
  <ol>
    <li><a href = "#summary">Summary</a></li>
    <li>
      <a href = "#about-our-project">About Our Project</a>
      <ul>
        <li><a href = "#inspiration">Inspiration</a></li>
        <li><a href = "#project-architecture">Technical Architecture</a></li>
      </ul>
    </li>
    <li><a href = "#about-us">About Us</a></li>
    <li>
      <a href = "#getting-started">Getting Started</a>
      <ul>
        <li><a href = "#setting-up">Set Up</a></li>
        <li><a href = "#running-instructions">How to Run?</a></li>
        <li><a href = "#testing-the-code">To Run Tests</a></li>
      </ul>
    </li>
  </ol>
</details>



<!--- Summary of presentation introduction --->
## Summary
Our project is a mission with its focus on the environment friendly usage of alternative fuels. In our project, we hope to provide a connection of alternate fuel stations which will make a path from the users starting location to his destination. The project functions as map but traverses only based on latitudes and longitudes while accounting for the distance of roads by being multiplied by a road curvature constant. This project will go to show the shortest path from one alternate fuel station to another. Through this project, we hope to showcase the limitations of automobiles which rely on these alternate fuels.


<!--- Technical architecture of project --->
## About Our Project
### Inspiration
The inspiration for this project comes from the genuine care we have towards the environment as well as looking at some of the United Nations sustainablity goals for the environment. There already exists several scientific documents which provide evidence of the existence of global warming. Among other forms of pollution, automobiles are also a source of pollution and alternate fuels are a potential solution to this problem. 


### Project Architecture
<div align = "center"> 
  <img src = "archi.png">
</div>
<br>
<p>
  The image above shows the architecture of the project where we started with the data and then used the KD-Tree to make the graph. The graph has an adjacency list and other functions which help us write other algorithms. The graph is used to write Dijkstra's algorithm which turns our graph into a shortest path tree. This tree is later used to write the algorithm for the shortest path between two nodes which is the output to the program. The other method runs the BFS Traversal which gives all the alternate fuel stations which can eventually be reached by the automobile in the given range of the automobile. This can also be shown through the program output.

</p>



<!--- Group members and their roles --->
## About Us
The four of us are a team of students who have met and developed ourselves through the CS-225 course. We are all passionate programs with different set of skills and we have come together nicely on this project. We have all grew up in cities and polltion and environment destruction has always been a major part of our lives, hence we have used backgrounds to come up with this project. While all four of us have previous knowledge of C++, which makes up the majority of our project, we also used python and other python libraries in order to make our data easy to read and use. Only Brendan and Aakash have previous experiences with python. 

The four of us also enjoyed working on this project together due to the innovative and new things we got to do throughout the project. Aside from python, we also attempted to use a K/D Tree to represent our data which was something we didn't envision ourselves doing. This part of the project was challenging and brought out some of our best work. 



<!--- Provides reproducible installation and running instructions --->
## Getting Started
### Setting Up

<ol type="1">
   <li>Open up your Terminal</li>
   <li><code>cd</code> into the folder containing cs225 Dockerfile</li>
   <li><code>git clone https://github.com/Ray-arch-debug/CS225-FinalProject.git</code></li>
   <li><code>cd CS225-FinalProject</code></li>
  <li><code>mkdir build</code></li>
  <li><code>cd build</code></li>
  <li><code>cmake ..</code></li>
</ol>



### Running Instructions

<ol type="1">
   <li>From the build directory, run <code>make</code></li>
  <li> <code> ./driver file_name fuel_type range starting_address 0/ending_address</code> </li>
  <li> The file_name indicates the data file you would like to use, the fuel_type indicates what kind of fuel your automobile uses, the range indicates the distance your car can travel in miles <insert as a double>, the address indicates the address of the alternate fuel station you would begin on, the number indicates which algorithm to run -> 0 means running the BFS and putting the address means running Dijkstra's algorithm which runs another algorithm to print the shortest path. </li>
   
</ol>




### Testing the code

<ol type="1">
   <li>From the build directory, run <code>make test</code></li>
  <li> Run <code>./test</code> to run all test cases, or run each individual test by running <code>./test</code> (test_case_name) </li>
  <li> Upon successful run, you???ll see the following message (it might take a while): </li>
  <img src = "test_output.png">
  
  <li> Note: Be mindful that you might fail a test case if your machine is slower than ours, though this should be ignored as the time limits are mostly arbitrary. </li>
</ol>





