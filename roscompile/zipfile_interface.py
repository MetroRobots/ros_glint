import requests
import shutil
from ros_introspection.ros_util import get_package_path


def locate_zip_file(filename='test_data.zip', branch='main'):
    # Workaround for catkin_download_test_data
    share_path = get_package_path('roscompile')
    target_path = share_path / filename
    if target_path.exists():
        return target_path

    url = f'https://github.com/DLu/roscompile_test_data/raw/{branch}/{filename}'
    req = requests.get(url, stream=True)
    with open(target_path, 'wb') as f:
        shutil.copyfileobj(req.raw, f)

    return target_path
