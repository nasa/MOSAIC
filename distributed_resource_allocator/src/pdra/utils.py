import importlib
import os
from datetime import datetime, timedelta, tzinfo

def load_class_dynamically(directory, module_name, class_name=None):
    """ Load a class dynamically.

        :param str dir: Directory where the module is located
        #:param str class_package: The entire class package (e.g. ``simulator.utils``)
        :param str module_name: The module name (e.g. ``DtnUtils``)
        :param str class_name: Class name within the module. If not provided, assumes same as module_name

        ..code::

            clazz  = find_class_dynamically('simulator.core', 'DtnCore', 'Simulable')
            bundle = clazz(*args, **kwargs)

        Bundle is now an instance of the class simulator.core.DtnCore.Bundle
    """
    # Set default values
    if not class_name:
        class_name = module_name

    # Change to the appropriate directory
    pwd = os.getcwd()
    os.chdir(os.path.abspath(directory))

    #module = importlib.import_module('{}.{}'.format(class_package, module_name))
    module = importlib.import_module(module_name)
    cls    = getattr(module, class_name)

    # Go back to the original directory
    os.chdir(pwd)

    return cls

# =====================================================================
# === Time-related functions. All defined in UTC
# =====================================================================

dt0 = timedelta(0)

class UTC(tzinfo):
    """ UTC time zone """
    def utcoffset(self, dt):
        return dt0

    def tzname(self, dt):
        return 'UTC'

    def dst(self, dt):
        return dt0

    def __str__(self):
        return 'UTC'

    def __repr__(self):
        return 'UTC'

t0 = datetime(1970, 1, 1, tzinfo=UTC())

def now():
    """ Returns the number of seconds elapsed since 01/01/1970 in UTC timezone. This is
        essentially equivalent to a POSIX timestamp

        :return: Timestamp as an integer
    """
    return int((datetime.now(UTC()) - t0).total_seconds())