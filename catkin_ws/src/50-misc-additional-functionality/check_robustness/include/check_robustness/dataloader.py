import os
import string

import cv2


def is_filetype(file, extension):
    return os.path.splitext(file)[1] == extension


class DataLoader:
    """
    This class sets a convention on how to data is stored within the whole package. It uses the very simple convention
    that all images must be stored in the same directory and labels (if they exist) must be written into a .txt file
    with the same name as the corresponding image. The label will be read by splitting the whole text at linebreaks and
    spaces and then every entry is tried to be converted to a float. As long as at least one can be converted, the
    label is accepted.
    """

    def __init__(self, path, has_ground_truth=False):
        """
        Loads a list of .jpg images from path into memory.

        :param path: str -> The root directory where images are searched.
        :param has_ground_truth: bool -> If True, every image requires a corresponding label file.
        """
        self.__n = 0
        self.has_ground_truth = has_ground_truth
        self.images = [os.path.join(path, file) for file in os.listdir(path) if is_filetype(file, ".jpg")]
        self.labels = []

        if has_ground_truth:
            for image in self.images:
                img_name = os.path.basename(image)
                lbl_name = "%s.txt" % string.join(img_name.split(".")[:-1])
                lbl_path = os.path.join(os.path.dirname(image), lbl_name)
                if not os.path.exists(lbl_path):
                    raise ValueError("Could not find the label file %s" % lbl_path)
                label = []
                with open(lbl_path, "r") as fid:
                    for line in fid.readlines():
                        for entry in line.strip().split(" "):
                            if entry:
                                try:
                                    label.append(float(entry))
                                except ValueError:
                                    pass

                if not label:
                    raise ValueError("Could not read the labels from the file %s" % lbl_path)

                self.labels.append(label)

    def __iter__(self):
        self.__n = 0
        return self

    def next(self):
        if self.__n < len(self.images):
            img = cv2.imread(self.images[self.__n])
            if img is None:
                self.__n += 1  # This is useful if you want to skip this file by catching the exception and continue.
                raise IOError("Cloud not read the images file: %s" % self.images[self.__n - 1])
            if self.has_ground_truth:
                self.__n += 1
                return img, self.labels[self.__n - 1]
            self.__n += 1
            return img
        else:
            raise StopIteration


if __name__ == "__main__":
    pass
