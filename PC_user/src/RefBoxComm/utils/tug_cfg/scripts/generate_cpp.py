#!/usr/bin/env python
import sys
from tug_cfg.loader import Loader
from tug_cfg import cpp


def main():
    #try:
        l = Loader()
        cfg = l.load(sys.argv[1], sys.argv[2])
        g = cpp.Generator()
        g.generate(sys.stdout, cfg)
    #except ValueError as e:
    #    print('Error: %s' % e.message)
    #except Exception as e:
    #    print('Error: %s' % e)

if __name__ == '__main__':
    main()
