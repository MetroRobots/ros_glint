from colorama import Fore, Back, Style, init
import shutil
import sys

init()


if sys.stdout.encoding == 'utf-8':
    TL = '╭'
    TR = '╮'
    BL = '╘'
    BR = '╛'
    TC = '─'
    BC = '═'
    VC = '│'
else:
    TL = TR = BL = BR = '+'
    TC = BC = '-'
    VC = '|'


def color_diff(diff, buffer_size=5):
    # Buffer size is the number of lines before and after diffs to print
    buffer = []
    started = False

    for line in diff:
        color = ''
        if line.startswith('+'):
            color = Fore.GREEN
        elif line.startswith('-'):
            color = Fore.RED
        elif line.startswith('^'):
            color = Fore.BLUE
        elif line.startswith('?'):
            continue

        if color:
            if buffer:
                if len(buffer) > buffer_size:
                    yield '...'
                yield from buffer[-buffer_size:]
            started = True
            yield color + line + Fore.RESET
            buffer = []
        else:
            if started and len(buffer) >= buffer_size:
                yield from buffer[:buffer_size]
                buffer = []
                started = False
            buffer.append(line)
    if buffer:
        yield from buffer[-buffer_size:]
        yield '...'


def color_header(s, fore='WHITE', back='BLUE'):
    ts = shutil.get_terminal_size()
    header = ''
    header += getattr(Fore, fore) + getattr(Back, back)
    header += TL + (TC * (ts.columns - 2)) + TR + '\n'
    header += VC + ' ' + Style.BRIGHT
    header += s.ljust(ts.columns - 3, ' ') + Style.NORMAL + VC + '\n'
    header += BL + (BC * (ts.columns - 2)) + BR
    header += Style.RESET_ALL
    return header


def color_text(s, fore='YELLOW', bright=False):
    return (Style.BRIGHT if bright else '') + getattr(Fore, fore) + s + Style.RESET_ALL


def query_yes_no(question, default='no'):
    """Ask a yes/no question via input() and return their answer.

    Based on http://code.activestate.com/recipes/577058/

    'question' is a string that is presented to the user.
    'default' is the presumed answer if the user just hits <Enter>.
        It must be 'yes' (the default), 'no' or None (meaning
        an answer is required of the user).

    The 'answer' return value is True for 'yes' or False for 'no'.
    """
    valid = {'yes': True, 'y': True, 'ye': True,
             'no': False, 'n': False}
    if default is None:
        prompt = ' [y/n] '
    elif default == 'yes':
        prompt = ' [Y/n] '
    elif default == 'no':
        prompt = ' [y/N] '
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        choice = input(color_text(question + prompt)).lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            print("Please respond with 'yes' or 'no' (or 'y' or 'n').")
