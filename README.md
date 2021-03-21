# AI2 Chess Assistant

## About

This project helps you with the help of computer vision and Ai with the optimal move to make in chess

## Requirements

To use this project it is required to install the dependencies with

```bash
pip install -e .
```

For develoment it is required to run.

```bash
pip install -e ".[development]"
```

If that doesnt work use `pip3` this is because you also have python 2 installed

## Advise

When developing in python it is best to create a virtual environment. to do this, run:

```bash
python3 -m venv .venv
```

this creates a `.venv` directory in your project. VScode will prompt you if you want to use this environment already.
To use this environment in the terminal you have to use `. .venv/bin/activate` and to leave this enviorment just type `deactivate`

## Docker

You can use your own way of running containers. My personal methon is

```bash
docker-compose up
docker-compose run python
```

and then just use python inside the container

## Handy findings

To have less clutter in your project you can remove the `.pyc` files and move them to another place in python versions > 3.8

```bash
 export PYTHONPYCACHEPREFIX="$HOME/.cache/pycache/"
```

## TODO

- implement ROS container
