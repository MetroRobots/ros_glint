import difflib
from .terminal import color_diff


def files_match(pkg_in, pkg_out, filename, show_diff=True):
    """Return true if the contents of the given file are the same in each package. Otherwise maybe show the diff."""
    generated_contents = pkg_in.get_contents(filename).rstrip()
    canonical_contents = pkg_out.get_contents(filename).rstrip()
    ret = generated_contents == canonical_contents
    if show_diff and not ret:
        d = difflib.Differ()
        print('=' * 50 + str(filename))
        result = d.compare(generated_contents.split('\n'), canonical_contents.split('\n'))
        print('\n'.join(color_diff(result)))
    return ret
