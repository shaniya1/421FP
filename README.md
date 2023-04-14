To help with setting up, on the command window on MATLAB, type in <edit .gitattributes>
Within that window, copy and paste everything down below. I believe this is MATLAB's version 
of extensions since I am not seeing git anywhere in the add-ons. Anyhow, that should allow us 
to implement source control. To do that, we need a create a blank folder within MATLAB first 
where all the project files will go. Don't worry if you have another folder with all the mass 
property code that we already worked on. You can always copy/paste and push to the main branch. 

% Basic .gitattributes for a MATLAB repo. 
% This template includes Simulink and MuPAD extensions, in addition to the MATLAB extensions. 

% Source files 
*.m text diff=matlab 
*.mu text diff=matlab 
  
% Caution: *.m also matches Mathematica packages. 
  
% Binary Files 
*.p binary 
*.mex* binary 
*.fig binary 
*.mat binary 
*.mdl binary 
*.slx binary 
*.mdlp binary 
*.slxp binary 
*.sldd binary 
*.mltbx binary
*.mlappinstall binary 
*.mlpkginstall binary 
*.mn binary 
