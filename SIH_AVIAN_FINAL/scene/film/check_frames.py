import glob, zlib, struct
from concurrent.futures import ProcessPoolExecutor
from PIL import Image
def check(f):
    try:
        b = open(f,'rb').read()
        if not b.startswith(b'\x89PNG\r\n\x1a\n'): return f, 'bad signature'
        if not b.endswith(b'IEND\xaeB`\x82'): return f, 'no IEND (truncated)'
        p = 8
        while p < len(b):
            n, = struct.unpack('>I', b[p:p+4])
            if zlib.crc32(b[p+4:p+8+n]) & 0xffffffff != struct.unpack('>I', b[p+8+n:p+12+n])[0]:
                return f, 'CRC mismatch'
            p += 12 + n
        im = Image.open(f); im.load()
        return (f, None) if im.size == (1920,1080) else (f, f'size {im.size}')
    except Exception as e:
        return f, repr(e)
if __name__ == '__main__':
    fs = sorted(glob.glob('*/f*.png')); bad=[]
    with ProcessPoolExecutor(8) as ex:
        for f, err in ex.map(check, fs, chunksize=8):
            if err: bad.append((f,err))
    print('checked', len(fs), 'bad', len(bad))
    for f,e in bad: print(' BAD', f, e)
