# #{ Exceptions and Errors
#  exceptions that can be raised by the spawner
class NoFreeIDAvailable(RuntimeError):
    """Raised when an ID could not be automatically assigned."""
    pass


class NoValidIDGiven(RuntimeError):
    """Raised when a user-provided ID is already in use."""
    pass


class CouldNotLaunch(RuntimeError):
    """Raised when a subprocess (like PX4 or MAVROS) fails to start."""
    pass


class CouldNotSpawn(RuntimeError):
    """Raised when a gazebo spawn call fails"""
    pass


class FormattingError(ValueError):
    """Raised when spawn arguments are in an unrecognizable format."""
    pass


class WrongNumberOfArguments(ValueError):
    """Raised when a command-line flag is missing its required value."""
    pass


class SuffixError(NameError):
    """Raised for issues related to file suffixes, like for Jinja templates."""
    pass


# #} end of Exceptions and Errors
