## Map Conversion

Make sure the ADP Python Wheel is installed in your local environment.
You can find this in the `simian_interface` folder of the ADP ship.
IMPORTANT: A given Simian wheel must be installed for a specific matching version of Python
(typically Python 3.7). This may differ from your default system version of Python3, but it is ok to
have multiple Python versions installed alongside each other.

```
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt install python3.7
sudo apt install python3.7-distutils
sudo pip3.7 install --upgrade pip
python3.7 -m pip install Cython pybind11 pythran
python3.7 -m pip install testresources numpy scipy shapely
python3.7 -m pip install <simian_ship>/adp_interface/simian_py-<version_string>.whl
```

Then you can run the conversion script:
```
python3.7 convert_map.py --input_file path/to/map --output_dir path/to/folder
```

If there are error messages about numpy or scipy, uninstall and reinstall the ADP wheel and the
offending packages, making sure to specify the Python version when installing (as shown above).
