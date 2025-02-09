Week0 Information
=========================

Please refer to canvas for video's to assist in conjunction with the exercises provided.

First and foremost, the subject is delivered on Linux OS (Ubuntu) so we initially discuss circumnavigating Linux. Then we move to CMake, which is the build system used, this let's you compile code and create libraries and executable(s). We then also introduce Visual Studio Code as Integrated Development Environment, that you write code in.

It is *ESSENTIAL* you have Linux installed and have compiled our sample code to be able to partake in Week 01 activities.

The below introduces

* Linux filesystem

* Bash operations

* cmake 

* compiling (out of source build)

* git (version control)

* Visual Studio Code - aka vscode (the IDE we will be using)

Consult the videos on canvas to achieve below.

Ex01 - Files on Linux
--------------------
* Create a directory (mkdir)
* Create a text file in it (use gedit and nano)
* Make a copy of the file and modify it (cp)
* Delete the original file
* Copy folder week00, which is inside your git respoitory, `tutorial` folder to the `scratch` folder inside your git repository

Ex02 - Using GIT
--------------------
* create a text file `EXAMPLE.md` in the `week00` folder, in your scratch folder
* add / commit and push the `EXAMPLE.md` to your git repoistory

Ex03 - Compiling cmake projects (command line only)
---------------------
Compile [hello.cpp](./starter/hello.cpp)

* change directory to starter
* create build directory 
* run cmake
* run make
* run executable 
* modify cout text to use your name "Hello <YOUR_NAME>"
* rebuild and run
* commit modified project (without build products) to your repository

Ex04- Compiling cmake projects (vscode)
---------------------
Compile [hello.cpp](./starter/hello.cpp)
* create build directory
* start vscode ``code``
* rebuild and run
