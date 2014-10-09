
Our algorithm use D1 data (grayscale + depth) Only.





Instruction on How to Run the Code


1) First please unzip the file and save it to some working directory.

2) Change the Matlab working directory to this directory, and you can see a Main_func.m (which is the main function) and a folder named 'All_func'.

3) We can running the testing procedure within this directory.

For example:

Suppose we want to test the video sequence 'vid0003'. Its full path is C:\xxx\vid0003
Then in Matlab command window, key in:

----------------------------------

Main_func('C:\xxx\vid0003');

-----------------------------------

4) The output xml file will be saved in the working directory.



Note:

1) Our script will call some pre-compiled .exe, which is using OpenCV package, although we have attached all the requried opencv dll and lib in the submitted package, you might encounter some system configuration error, if that happens, please let us know and we will try to help you configure it correctly.

2) Our program is mostly written in Matlab, so it is a little bit slow, for a 1000 frames video sample, it might take 30 mins to run the testing.

3) Our program will first run some human pose detector, then run depth-gray mapping and then analyze the whole sequence and localize the actions.

4) We use D1 data only: Grayscale + depth.





Team Contact Information

Team Name: ADSC-NUS-Singapore

1. Bingbing Ni
Email: bingbing.ni@adsc.com.sg

Advanced Digital Sciences Center (ADSC), Singapore
1 Fusionopolis Way, Connexis North Tower 08-10, Singapore 138632

2. Yong Pei
Email: pei.yong@adsc.com.sg

Advanced Digital Sciences Center (ADSC), Singapore
1 Fusionopolis Way, Connexis North Tower 08-10, Singapore 138632

3. Jun Tan
Email: tanjun.nudt@gmail.com

National University of Singapore

4. Jian Dong
Email: a0068947@nus.edu.sg

National University of Singapore

5. Shuicheng Yan
Email: eleyans@nus.edu.sg

National University of Singapore

6. Pierre Moulin
Email: moulin@ifp.uiuc.edu

University of Illinois at Urbana-Champaign