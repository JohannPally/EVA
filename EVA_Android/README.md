# EVA Android application
This folder contains all the necessary files to run the EVA android application on an Android phone with version 6.0 or above
In order to run this component, several steps needs to be done

	1. Create a new Android project in Android Studio and copy the files directly in
	
	2. Change the IP addr and port number to be the cloud server address and port in MainActivity.java This is the only configurations that needs the be done
	
	3. Either connect an Android phone or start an amulator within Android Studio. 
	
	4. If ran once successfully on an physical phone, it means the application is installed and can be unplugged
	
	5. Make sure cloud server code is running before starting connection
	
	6. Push "Start Connection" to start image transmission with the cloud server
	
	7. Push "Close Connection" to stop the image transmission
	
In addition to the instructions, make sure the IP address of both the phone and cloud server are discoverable to each other.
For higher performance, ensure the internet connection has a fairly steady high speed
Remember to place phone in a steady location for better pose estimation performance
