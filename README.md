# Privacy-Preserving Trajectory Matching for Unmanned Aerial Vehicles Proof-of-Concept
Proof-of-concept of PPTM running between a server and a client.

## Bootstrapping / setting up for development
- Install Python 3.8 or higher
- Create a virtual environment: `python -m venv .venv`
- Activate the virtual environment 
  - On Unix or MacOS, using the bash shell: `source /path/to/venv/bin/activate`
  - On Unix or MacOS, using the csh shell: `source /path/to/venv/bin/activate.csh`
  - On Unix or MacOS, using the fish shell: `source /path/to/venv/bin/activate.fish`
  - On Windows using the Command Prompt: `path\to\venv\Scripts\activate.bat`
  - On Windows using PowerShell: `path\to\venv\Scripts\Activate.ps1`
- Install required dependencies by running `pip install -r requirements.txt` or `python -m pip install -r requirements.txt`

### Adding packages to requirements.txt
- First, initialize for development using the steps above
- Next, install a package by running `pip install {packagename}`
- Install multiple packages if required.
- Next, write the dependency packages to the requirements file: `pip freeze > requirements.txt`
- Don't forget to commit changes to requirements.txt

### How to setup Ad Hoc Connection on Windows
- Open Command Prompt (Admin)
- To ensure that your network interface supports the Hosted Network feature, type the following command and press Enter: `netsh wlan show driver`
- If “Hosted network supported” shows “Yes”, proceed to step 4. If it says “No”, you can try to update your wireless driver. If the updated driver still does not help, then you will need to upgrade your hardware.
- To configure Ad Hoc Wireless connection, type this command in the command prompt and press Enter: `netsh wlan set hostednetwork mode=allow ssid=<your desired network name> key=<your desired password>`
- To start your new network, type this command and press Enter: `netsh wlan start hostednetwork`
- Navigate to Control Panel > Network and Sharing Center. Click on “Change adapter settings”
- Now connect with the other device to the created Wi-Fi network

### How to run
- setup connection between the client device and the server device (if you want to run on two devices)
- Set the server_ipv4_address in client.py and server.py to the ip address of the device that will be the server. (Set to 'localhost' for run on the same device)
- Run client.py on the client device and run server.py on the server device. (or both on the same device)