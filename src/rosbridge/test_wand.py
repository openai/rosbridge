import wand.image

def test_loader(prefix):
    print 'Prefix=%s...' % prefix
    data = prefix + ('\0' * (640*480*3-len(prefix)))

    im = wand.image.Image(blob=data, width=640, height=480, format='rgb', depth=8)

    print 'Prefix=%s: ok' % prefix

test_loader('abc')
test_loader('%!PS')
