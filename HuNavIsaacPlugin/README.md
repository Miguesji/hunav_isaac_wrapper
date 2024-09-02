# Loading Extension
To enable this extension, run Isaac Sim with the flags --ext-folder {path_to_ext_folder} --enable {ext_directory_name}
The user will see the extension appear on the toolbar on startup with the title they specified in the Extension Generator


# Extension Usage
With HuNav-Isaac Plugin you can use HuNavSim in Isaac Sim to run realistic simulations of humans that interact with obstacles and robots.


# Template Code Overview
The template is well documented and is meant to be self-explanatory to the user should they
start reading the provided python files.  A short overview is also provided here:

global_variables.py: 
    A script that stores in global variables that the user specified when creating this extension such as the Title and Description.

extension.py:
    A class containing the code to comunitace with HuNavSim and control the human agents.
    In extension.py, useful standard callback functions are created that the user may complete in ui_builder.py.

ui_builder.py:
    Controls the user interface.

world_builder.py:
    In this file you can find all the necessary to build an environment for your simulations.
