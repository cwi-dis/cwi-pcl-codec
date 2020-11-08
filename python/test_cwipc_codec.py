import unittest
import cwipc
import _cwipc_codec
import os
import sys
import tempfile

if 0:
    # This code can be used to debug the C++ code in XCode:
    # - build for XCode with cmake
    # - build the cwipc_util project
    # - Fix the pathname for the dylib
    # - run `python3 test_cwipc_util`
    # - Attach to python in the XCode debugger
    # - press return to python3.
    import _cwipc_codec
    _cwipc_codec._cwipc_codec_dll('/Users/jack/src/VRTogether/cwipc_codec/build-xcode/lib/Debug/libcwipc_codec.dylib')
    print('Type return after attaching in XCode debugger - ')
    sys.stdin.readline()

#
# Windows search path is horrible. Work around it for testing with an environment variable
#
if 'CWIPC_TEST_DLL' in os.environ:
	filename = os.environ['CWIPC_TEST_DLL']
	dllobj = _cwipc_codec._cwipc_codec_dll(filename)
#
# Find directories for test inputs and outputs
#
if 'CWIPC_TEST_DIR' in os.environ:
    CWIPC_TEST_DIR=os.environ['CWIPC_TEST_DIR']
else:
    _thisdir=os.path.dirname(os.path.abspath(__file__))
    _topdir=os.path.dirname(_thisdir)
    TEST_FIXTURES_DIR=os.path.join(_topdir, "tests", "fixtures")
    TEST_OUTPUT_DIR=os.path.join(TEST_FIXTURES_DIR, "output")
    if not os.access(TEST_OUTPUT_DIR, os.W_OK):
        TEST_OUTPUT_DIR=tempfile.mkdtemp('cwipc_codec_test')
PLY_FILENAME=os.path.join(TEST_FIXTURES_DIR, "input", "pcl_frame1.ply")
COMPRESSED_FILENAME=os.path.join(TEST_FIXTURES_DIR, "compressed", "pcl_frame1.cwicpc")

class TestApi(unittest.TestCase):
        
    def test_cwipc_new_encoder(self):
        """Can we create and free a cwipc_encoder object"""
        encoder = _cwipc_codec.cwipc_new_encoder()
        self.assertFalse(encoder.eof())
        self.assertFalse(encoder.available(False))
        encoder.free()
        
    def test_cwipc_new_encoder_bad_parameters(self):
        """Do we get exceptions when creating an encoder with unimplemented parameters?"""
        with self.assertRaises(cwipc.CwipcError):
            encoder = _cwipc_codec.cwipc_new_encoder(do_inter_frame=True)
        with self.assertRaises(cwipc.CwipcError):
            encoder = _cwipc_codec.cwipc_new_encoder(gop_size=0)
        with self.assertRaises(cwipc.CwipcError):
            encoder = _cwipc_codec.cwipc_new_encoder(gop_size=2)
        
    def test_cwipc_encoder_close(self):
        """Can we close a encoder"""
        encoder = _cwipc_codec.cwipc_new_encoder()
        self.assertFalse(encoder.eof())
        self.assertFalse(encoder.available(False))
        encoder.close()
        self.assertTrue(encoder.eof())
        self.assertFalse(encoder.available(True))
        encoder.free()

    def test_cwipc_new_decoder(self):
        """Can we create and free a cwipc_decoder object"""
        decoder = _cwipc_codec.cwipc_new_decoder()
        self.assertFalse(decoder.eof())
        self.assertFalse(decoder.available(False))
        decoder.free()

    def test_cwipc_decoder_close(self):
        """Can we close a decoder"""
        decoder = _cwipc_codec.cwipc_new_decoder()
        self.assertFalse(decoder.eof())
        self.assertFalse(decoder.available(False))
        decoder.close()
        self.assertTrue(decoder.eof())
        self.assertFalse(decoder.available(True))
        decoder.free()

    def test_cwipc_encoder_plyfile(self):
        """Test that we can encode a PLY file and the available flags behaves correct"""
        pc = cwipc.cwipc_read(PLY_FILENAME, 1234)
        self._verify_pointcloud(pc)
        encoder = _cwipc_codec.cwipc_new_encoder()
        self.assertFalse(encoder.available(False))
        encoder.feed(pc)
        self.assertTrue(encoder.available(False))
        self.assertTrue(encoder.at_gop_boundary())
        data = encoder.get_bytes()
        self.assertNotEqual(len(data), 0)
        self.assertFalse(encoder.available(False))
        encoder.free()
        pc.free()        

    @unittest.skip('unexpected difference')
    def test_cwipc_encoder_plyfile_multiple_same(self):
        """Test that we can encode a ply file multiple times and the results are the same"""
        pc = cwipc.cwipc_read(PLY_FILENAME, 1234)
        encoder = _cwipc_codec.cwipc_new_encoder()
        encoder.feed(pc)
        data_one = encoder.get_bytes()
        encoder.feed(pc)
        data_two = encoder.get_bytes()
        self.assertEqual(data_one, data_two)
                
        encoder.free()
        pc.free()

    def test_cwipc_encoder_plyfile_multiple_different(self):
        """Test that we can encode a ply file multiple times and the results are the same, and different for different timestamps"""
        pc = cwipc.cwipc_read(PLY_FILENAME, 1234)
        encoder = _cwipc_codec.cwipc_new_encoder()
        encoder.feed(pc)
        data_one = encoder.get_bytes()
        pc3 = cwipc.cwipc_read(PLY_FILENAME, 4567)
        encoder.feed(pc3)
        data_three = encoder.get_bytes()
        self.assertNotEqual(data_one, data_three)
        
        encoder.free()
        pc.free()
        pc3.free()
        
    def test_cwipc_encoder_octree_depth(self):
        """Test that octree_bits encoder param makes a significant difference"""
        pc = cwipc.cwipc_read(PLY_FILENAME, 1234)
        decoder = _cwipc_codec.cwipc_new_decoder()
        depth = 11
        decoded_npoints_per_depth = {}
        cellsize_per_depth = {}
        while depth >= 0:
            encoder = _cwipc_codec.cwipc_new_encoder(octree_bits=depth)
            encoder.feed(pc)
            encoded_data = encoder.get_bytes()
            encoded_size = len(encoded_data)
            decoder.feed(encoded_data)
            assert decoder.available(True)
            decoded_pc = decoder.get()
            points = decoded_pc.get_points()
            decoded_npoints = len(points)
            decoded_npoints_per_depth[depth] = decoded_npoints
            cellsize_per_depth[depth] = decoded_pc.cellsize()
            encoder.free()
            decoded_pc.free()
            depth = depth - 1
        # Some sanity checks
        for i in range(11):
            self.assertLessEqual(decoded_npoints_per_depth[i], decoded_npoints_per_depth[i+1])
        for i in range(11):
            self.assertGreater(cellsize_per_depth[i], cellsize_per_depth[i+1])
        if decoded_npoints_per_depth[11] < 80000:
            self.assertLessEqual(decoded_npoints_per_depth[0], 16)
        else:
            self.assertLessEqual(decoded_npoints_per_depth[0], decoded_npoints_per_depth[11]/10000)
        pc.free()
        
    def test_cwipc_encoder_jpeg_quality(self):
        """Test that jpeg_quality encoder param makes a difference"""
        pc = cwipc.cwipc_read(PLY_FILENAME, 1234)
        quality = 90
        prev_size = None
        while quality >= 10:
            params = _cwipc_codec.cwipc_new_encoder_params(jpeg_quality=quality)
            encoder = _cwipc_codec.cwipc_new_encoder(params=params)
            encoder.feed(pc)
            encoded_data = encoder.get_bytes()
            encoded_size = len(encoded_data)
            if prev_size != None:
                self.assertLess(encoded_size, prev_size)
            prev_size = encoded_size
            encoder.free()
            quality = quality - 10
        pc.free()
        
    def test_cwipc_decode_cwicpc(self):
        """Test that we can decode a cwicpc compressed pointcloud and get an acceptable cwipc"""
        with open(COMPRESSED_FILENAME, 'rb') as fp:
            compdata = fp.read()
        decoder = _cwipc_codec.cwipc_new_decoder()
        self.assertFalse(decoder.available(False))
        decoder.feed(compdata)
        self.assertTrue(decoder.available(False))
        pc = decoder.get()
        self.assertFalse(decoder.available(False))
        self._verify_pointcloud(pc)
        decoder.free()
        pc.free()
        
    def test_cwipc_codec_roundtrip(self):
        """Check that we can roundtrip encoder-decoder and get at most as many points back"""
        timestamp = 0x1122334455667788
        pc = cwipc.cwipc_read(PLY_FILENAME, timestamp)
        encoder = _cwipc_codec.cwipc_new_encoder()
        decoder = _cwipc_codec.cwipc_new_decoder()
        encoder.feed(pc)
        data = encoder.get_bytes()
        decoder.feed(data)
        pc2 = decoder.get()
        self._verify_pointcloud(pc2)
        points = pc.get_points()
        points2 = pc2.get_points()
        self.assertGreaterEqual(len(points), len(points2))
        self.assertEqual(pc2.timestamp(), timestamp)
        encoder.free()
        decoder.free()
        pc.free()     
        pc2.free()   

    def test_cwipc_codec_roundtrip_empty(self):
        """Check that we can roundtrip an empty pointcloud"""
        timestamp = 0x2233445566778899
        pc = cwipc.cwipc_from_points([], timestamp)
        encoder = _cwipc_codec.cwipc_new_encoder()
        decoder = _cwipc_codec.cwipc_new_decoder()
        encoder.feed(pc)
        data = encoder.get_bytes()
        self.assertNotEqual(len(data), 0)
        decoder.feed(data)
        pc2 = decoder.get()
        points = pc.get_points()
        points2 = pc2.get_points()
        self.assertEqual(len(points), 0)
        self.assertEqual(len(points2), 0)
        self.assertEqual(pc2.timestamp(), timestamp)
        encoder.free()
        decoder.free()
        pc.free()     
        pc2.free()   
                
    def test_cwipc_multiencoder_octree_depth(self):
        """Test that a multiencoder setup for 2 octree depths returns sensible pointclouds"""
        gen = cwipc.cwipc_synthetic()
        pc_orig = gen.get()
        count_orig = len(pc_orig.get_points())
        group = _cwipc_codec.cwipc_new_encodergroup()
        enc_high = group.addencoder(octree_bits=9)
        enc_low = group.addencoder(octree_bits=5)
        group.feed(pc_orig)
        self.assertTrue(enc_high.available(False))
        self.assertTrue(enc_low.available(False))

        data_high = enc_high.get_bytes()
        data_low = enc_low.get_bytes()
        self.assertLess(len(data_low), len(data_high))
        
        # xxxjack todo: decompress both and compare
        group.free()
        pc_orig.free()        

    def _verify_pointcloud(self, pc):
        points = pc.get_points()
        self.assertGreater(len(points), 1)
        p0 = points[0].x, points[0].y, points[0].z
        p1 = points[len(points)-1].x, points[len(points)-1].y, points[len(points)-1].z
        self.assertNotEqual(p0, p1)
   
if __name__ == '__main__':
    unittest.main()
