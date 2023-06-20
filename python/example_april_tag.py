from mirte_duckietown.duckietown import Camera
from mirte_duckietown._common import id_to_sign

processor = Camera()

while True:
  tmp = processor.getAprilTag()
  print('Tags length: ', len(tmp))
  for tag in tmp:
    print('id: ', tag.id, 'tag: ', id_to_sign(tag.id))
  print('end tags')
