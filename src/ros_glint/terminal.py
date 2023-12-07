from colorama import Fore, Style, init

init()


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


def color_text(s, fore='YELLOW', bright=False):
    return (Style.BRIGHT if bright else '') + getattr(Fore, fore) + s + Style.RESET_ALL
