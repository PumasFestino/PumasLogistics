#!/usr/bin/env python


class YamlHelper():
    @staticmethod
    def get_param(config, name, default=KeyError):
        if name in config:
            return config[name]

        if default is KeyError:  # 'None' should be a valid default value, so we use something else
            raise KeyError("'%s' not found in config!" % name)

        return default

    @staticmethod
    def get_param_with_default(config, name, default):
        return YamlHelper.get_param(config, name, default=default)

    @staticmethod
    def has_key(config, name):
        return name in config

    @staticmethod
    def has_param(config, name):
        return name in config
