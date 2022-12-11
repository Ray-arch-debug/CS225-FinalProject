<div align = "center">
  
  <img src = "images/Run4.png">
  
  <p align = "center">
    <h1>RUN 4</h1>
    <p><h3>Created by Aakash Kumar, Brendan Biernacki, Shaurya ----, and Alex Wang</h3></p>
  </p>
  
  <p align = "center">
    <a href = "https://github.com/Ray-arch-debug/CS225-FinalProject/blob/main/documents/Team%20Proposal">Project Proposal</a>
    |
    <a href = "https://docs.google.com/document/d/1tTfg5maTvJ1vPNpdlG3aSTgqO196-tSzHHWcsHte4dg/edit?usp=sharing">Presentation Video</a>
    |
    <a href = "https://mediaspace.illinois.edu/media/t/1_501wkwbs">Data</a>
    |
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
        <li><a href = "#setting-up">Notes On Gameplay</a></li>
        <li><a href = "#running-instructions">Download Instructions</a></li>
        <li><a href = "#testing-the-code">Notes On Gameplay</a></li>
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
  <img src = "images/diagram.png">
</div>
<br>
<p align = "center">

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
   <li><code>cd CS-225FinalProject</code></li>
  <li><code>mkdir build</code></li>
  <li><code>cd build</code></li>
  <li><code>cmake ..</code></li>
</ol>



### Running Instructions

<ol type="1">
   <li>From the build directory, run <code>make main</code></li>
   
</ol>




### Testing the code

<ol type="1">
   <li>From the build directory, run <code>make test</code></li>
  <li> Run <code>./test</code> to run all test cases, or run each individual test by running <code>./test</code> (test_case_name) </li>
  <li> Upon successful run, you’ll see the following message (it might take a while): </li>
</ol>





