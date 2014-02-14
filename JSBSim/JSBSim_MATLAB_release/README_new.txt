
*** PLEASE NOTE- If you do not have either the Visual Studio 2008 Pro or Express editions, you may want to install either one. Some users have reported some problems running the compiled MEX file, but that by installing VS8 the SFunction ran fine. 

To start-

Type "jsbsimgui_start" in the Matlab command window. This will open the JSBSimGUI.fig file. With the GUI open, either type the name of your simulink model or leave "jsbsimgui_test" just as it appears in the textbox. Press "Load Model" to open the Simulink model. The model must be initialized before it can be run- do this by pressing "Initialize model".  If "Perform initial trim" is selected, then the model will be trimmed to the states present in the GUI upon pressing the "Initialize model" button.  Press the "Help" button for further information on how to use the GUI.

You can run any JSBSim aircraft model as long as it resides in the JSBSim aircraft data folder and it has an engine file in the engine folder.

NOTE: Currently the SFunction is based on the JSBSim Release Candidate 1.0 rc2, not the newer CVS versions. Sorry if that is a major problem for anyone. Once the CVS changes are in a release version, the SFunction will be updated to work with the latest release version. 

5/25/10
Brian Mills

