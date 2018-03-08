import sys

def run_pyreverse():
    """run pyreverse"""
    from pylint.pyreverse.main import Run
    args = sys.argv
    print(args)
    Run(args)

run_pyreverse()