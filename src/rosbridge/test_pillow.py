import time
import PIL.Image

def test_loader(prefix):
    print 'Prefix=%s...' % prefix
    t0 = time.time()
    data = prefix + ('\0' * (640*480*3-len(prefix)))

    im = PIL.Image.frombytes('RGB', (640, 480), data, 'raw')
    t1 = time.time()
    print 'Prefix=%s: ok %s (%0.6f)' % (prefix, im.size, t1-t0)

test_loader('abc')
test_loader('%!PS')
