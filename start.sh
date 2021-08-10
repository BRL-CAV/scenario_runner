python my_scenario_runner.py --openscenario srunner/examples/BRL/OpenSCENARIO_manual.xosc &

sleep 2

python manual_control.py &

sleep 5

python screen.py -c left -d 1 &

sleep 1
python screen.py -c center -d 2 &

sleep 1
python screen.py -c right -d 3 &

