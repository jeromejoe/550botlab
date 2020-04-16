"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import pose_xyt_t

class particle_t(object):
    __slots__ = ["pose", "parent_pose", "weight"]

    __typenames__ = ["pose_xyt_t", "pose_xyt_t", "double"]

    __dimensions__ = [None, None, None]

    def __init__(self):
        self.pose = pose_xyt_t()
        self.parent_pose = pose_xyt_t()
        self.weight = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(particle_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.pose._get_packed_fingerprint() == pose_xyt_t._get_packed_fingerprint()
        self.pose._encode_one(buf)
        assert self.parent_pose._get_packed_fingerprint() == pose_xyt_t._get_packed_fingerprint()
        self.parent_pose._encode_one(buf)
        buf.write(struct.pack(">d", self.weight))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != particle_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return particle_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = particle_t()
        self.pose = pose_xyt_t._decode_one(buf)
        self.parent_pose = pose_xyt_t._decode_one(buf)
        self.weight = struct.unpack(">d", buf.read(8))[0]
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if particle_t in parents: return 0
        newparents = parents + [particle_t]
        tmphash = (0x18016676f01f14e8+ pose_xyt_t._get_hash_recursive(newparents)+ pose_xyt_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if particle_t._packed_fingerprint is None:
            particle_t._packed_fingerprint = struct.pack(">Q", particle_t._get_hash_recursive([]))
        return particle_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

