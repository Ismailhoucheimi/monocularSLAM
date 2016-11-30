### USER SETTINGS ################################
folder = './media/ForwardMovement2/'
vid = 'ForwardMovementID-2.mp4'
images = ['01','02','03','04']#,'05','06','07','08']
imageType = '.png'
##################################################


videoName = folder+vid
imagesList = []
for name in images:
    imagesList.append(folder+name+imageType)
