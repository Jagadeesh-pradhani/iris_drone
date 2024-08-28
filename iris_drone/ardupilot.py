import subprocess
import sys

def run_sim_vehicle():
    # Command to execute
    command = ["sim_vehicle.py", "-v", "ArduCopter", "-f", "gazebo-iris", "--console"]

    try:
        # Execute the command and stream the output to the terminal
        process = subprocess.Popen(command, stdout=sys.stdout, stderr=sys.stderr)

        # Wait for the command to complete
        process.wait()
        
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    run_sim_vehicle()
