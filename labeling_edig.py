import os


def convert_images_from_folder(folder):
    for filename in os.listdir(folder):
        if filename[-3:] == "txt":
            file_ = open(os.path.join(folder, filename), 'r')
            lines = file_.readlines()
            file_.close()
            file_ = open(os.path.join(folder, "edited_"+filename), 'w')
            for line in lines:
                if int(line[0]) == 1:
                    print(line)
                if int(line[0]) == 0:
                    file_.write(line)
                if int(line[0]) > 1: 
                    newline = line[1:]
                    newline = str(int(line[0])-1) + newline
                    file_.write(newline)
            file_.close()

convert_images_from_folder("./test")
