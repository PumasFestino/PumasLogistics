
class Type(object):
    pass


class ScalarType(Type):
    #ANY = 'any'
    BOOL = 'bool'
    DOUBLE = 'double'
    INT = 'int'
    STR = 'str'
    ALL_NAMES = (#ANY,
                 BOOL, DOUBLE, INT, STR)

    def __init__(self, name):
        if name not in self.ALL_NAMES:
            raise ValueError("Invalid name for %s" % self.__class__.__name__)
        self.name = name


class CfgType(Type):
    def __init__(self, package_name, cfg_name):
        self.package_name = package_name
        self.cfg_name = cfg_name


class ListType(Type):
    def __init__(self, item_type, min_length=0, max_length=None):
        self.item_type = item_type
        self.min_length = min_length
        self.max_length = max_length


class MapType(Type):
    def __init__(self, key_type, value_type):
        self.key_type = key_type
        self.value_type = value_type


class Cfg(object):
    def __init__(self, package_name, name):
        super(Cfg, self).__init__()
        self.package_name = package_name
        self.name = name
        self.parameters = []


class Param(object):
    def __init__(self, name):
        super(Param, self).__init__()
        self.name = name
        self.type_name = None
        self.type = None
        self.unit = None
        self.default = None
        self.description = None
        self.min = None
        self.max = None
        self.dynamic = None
        self.level = None
        self.choices = None
        self.suggestions = None
        self.ignored = False
