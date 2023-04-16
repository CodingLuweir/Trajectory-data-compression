import os
import time
import zipfile


def zip_compress(filename: str) -> list:
    f = zipfile.ZipFile(filename[:-3] + "zip", 'w')
    f.write(filename,
            compress_type=zipfile.ZIP_DEFLATED)
    before_size = os.stat(filename).st_size
    after_zip_size = os.stat(filename[:-3] + "zip").st_size
    print("before_size: ", round(before_size / 1024, 2), "kb")
    print("after_zip_size: ", round(after_zip_size / 1024, 2), " kb")
    return [round(before_size / 1024, 2), round(after_zip_size / 1024, 2)]


def zip_decompress(filename: str) -> float:
    """
    返回用 zip 解压filename所需要的时间
    :param filename:
    :return:
    """
    decompress_start_time = time.perf_counter()
    zf = zipfile.ZipFile(filename, 'r')
    zf.extractall("../Restore_data")
    decompress_end_time = time.perf_counter()
    return decompress_end_time - decompress_start_time


if __name__ == '__main__':
    print(zip_decompress(
        r"E:\Desktop\Programmer\PythonFile\PythonProject\Experiment\TrajStore\Test\output_compressed_trajectory.zip"))
