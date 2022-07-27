/****************************************************************************
 * crypto/rijndael.c
 * $OpenBSD: rijndael.c,v 1.20 2014/11/17 12:27:47 mikeb Exp $
 *
 * rijndael-alg-fst.c
 *
 * @version 3.0 (December 2000)
 *
 * Optimised ANSI C code for the Rijndael cipher (now AES)
 *
 * @author Vincent Rijmen <vincent.rijmen@esat.kuleuven.ac.be>
 * @author Antoon Bosselaers <antoon.bosselaers@esat.kuleuven.ac.be>
 * @author Paulo Barreto <paulo.barreto@terra.com.br>
 *
 * This code is hereby placed in the public domain.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ''AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/param.h>
#include <sys/systm.h>

#include <crypto/rijndael.h>

#undef FULL_UNROLL

/* TE0[x] = S [x].[02, 01, 01, 03];
 * TE1[x] = S [x].[03, 02, 01, 01];
 * TE2[x] = S [x].[01, 03, 02, 01];
 * TE3[x] = S [x].[01, 01, 03, 02];
 *
 * TD0[x] = Si[x].[0e, 09, 0d, 0b];
 * TD1[x] = Si[x].[0b, 0e, 09, 0d];
 * TD2[x] = Si[x].[0d, 0b, 0e, 09];
 * TD3[x] = Si[x].[09, 0d, 0b, 0e];
 * TD4[x] = Si[x].[01];
 */

static const uint32_t TE0[256] =
{
  0xc66363a5u, 0xf87c7c84u, 0xee777799u, 0xf67b7b8du,
  0xfff2f20du, 0xd66b6bbdu, 0xde6f6fb1u, 0x91c5c554u,
  0x60303050u, 0x02010103u, 0xce6767a9u, 0x562b2b7du,
  0xe7fefe19u, 0xb5d7d762u, 0x4dababe6u, 0xec76769au,
  0x8fcaca45u, 0x1f82829du, 0x89c9c940u, 0xfa7d7d87u,
  0xeffafa15u, 0xb25959ebu, 0x8e4747c9u, 0xfbf0f00bu,
  0x41adadecu, 0xb3d4d467u, 0x5fa2a2fdu, 0x45afafeau,
  0x239c9cbfu, 0x53a4a4f7u, 0xe4727296u, 0x9bc0c05bu,
  0x75b7b7c2u, 0xe1fdfd1cu, 0x3d9393aeu, 0x4c26266au,
  0x6c36365au, 0x7e3f3f41u, 0xf5f7f702u, 0x83cccc4fu,
  0x6834345cu, 0x51a5a5f4u, 0xd1e5e534u, 0xf9f1f108u,
  0xe2717193u, 0xabd8d873u, 0x62313153u, 0x2a15153fu,
  0x0804040cu, 0x95c7c752u, 0x46232365u, 0x9dc3c35eu,
  0x30181828u, 0x379696a1u, 0x0a05050fu, 0x2f9a9ab5u,
  0x0e070709u, 0x24121236u, 0x1b80809bu, 0xdfe2e23du,
  0xcdebeb26u, 0x4e272769u, 0x7fb2b2cdu, 0xea75759fu,
  0x1209091bu, 0x1d83839eu, 0x582c2c74u, 0x341a1a2eu,
  0x361b1b2du, 0xdc6e6eb2u, 0xb45a5aeeu, 0x5ba0a0fbu,
  0xa45252f6u, 0x763b3b4du, 0xb7d6d661u, 0x7db3b3ceu,
  0x5229297bu, 0xdde3e33eu, 0x5e2f2f71u, 0x13848497u,
  0xa65353f5u, 0xb9d1d168u, 0x00000000u, 0xc1eded2cu,
  0x40202060u, 0xe3fcfc1fu, 0x79b1b1c8u, 0xb65b5bedu,
  0xd46a6abeu, 0x8dcbcb46u, 0x67bebed9u, 0x7239394bu,
  0x944a4adeu, 0x984c4cd4u, 0xb05858e8u, 0x85cfcf4au,
  0xbbd0d06bu, 0xc5efef2au, 0x4faaaae5u, 0xedfbfb16u,
  0x864343c5u, 0x9a4d4dd7u, 0x66333355u, 0x11858594u,
  0x8a4545cfu, 0xe9f9f910u, 0x04020206u, 0xfe7f7f81u,
  0xa05050f0u, 0x783c3c44u, 0x259f9fbau, 0x4ba8a8e3u,
  0xa25151f3u, 0x5da3a3feu, 0x804040c0u, 0x058f8f8au,
  0x3f9292adu, 0x219d9dbcu, 0x70383848u, 0xf1f5f504u,
  0x63bcbcdfu, 0x77b6b6c1u, 0xafdada75u, 0x42212163u,
  0x20101030u, 0xe5ffff1au, 0xfdf3f30eu, 0xbfd2d26du,
  0x81cdcd4cu, 0x180c0c14u, 0x26131335u, 0xc3ecec2fu,
  0xbe5f5fe1u, 0x359797a2u, 0x884444ccu, 0x2e171739u,
  0x93c4c457u, 0x55a7a7f2u, 0xfc7e7e82u, 0x7a3d3d47u,
  0xc86464acu, 0xba5d5de7u, 0x3219192bu, 0xe6737395u,
  0xc06060a0u, 0x19818198u, 0x9e4f4fd1u, 0xa3dcdc7fu,
  0x44222266u, 0x542a2a7eu, 0x3b9090abu, 0x0b888883u,
  0x8c4646cau, 0xc7eeee29u, 0x6bb8b8d3u, 0x2814143cu,
  0xa7dede79u, 0xbc5e5ee2u, 0x160b0b1du, 0xaddbdb76u,
  0xdbe0e03bu, 0x64323256u, 0x743a3a4eu, 0x140a0a1eu,
  0x924949dbu, 0x0c06060au, 0x4824246cu, 0xb85c5ce4u,
  0x9fc2c25du, 0xbdd3d36eu, 0x43acacefu, 0xc46262a6u,
  0x399191a8u, 0x319595a4u, 0xd3e4e437u, 0xf279798bu,
  0xd5e7e732u, 0x8bc8c843u, 0x6e373759u, 0xda6d6db7u,
  0x018d8d8cu, 0xb1d5d564u, 0x9c4e4ed2u, 0x49a9a9e0u,
  0xd86c6cb4u, 0xac5656fau, 0xf3f4f407u, 0xcfeaea25u,
  0xca6565afu, 0xf47a7a8eu, 0x47aeaee9u, 0x10080818u,
  0x6fbabad5u, 0xf0787888u, 0x4a25256fu, 0x5c2e2e72u,
  0x381c1c24u, 0x57a6a6f1u, 0x73b4b4c7u, 0x97c6c651u,
  0xcbe8e823u, 0xa1dddd7cu, 0xe874749cu, 0x3e1f1f21u,
  0x964b4bddu, 0x61bdbddcu, 0x0d8b8b86u, 0x0f8a8a85u,
  0xe0707090u, 0x7c3e3e42u, 0x71b5b5c4u, 0xcc6666aau,
  0x904848d8u, 0x06030305u, 0xf7f6f601u, 0x1c0e0e12u,
  0xc26161a3u, 0x6a35355fu, 0xae5757f9u, 0x69b9b9d0u,
  0x17868691u, 0x99c1c158u, 0x3a1d1d27u, 0x279e9eb9u,
  0xd9e1e138u, 0xebf8f813u, 0x2b9898b3u, 0x22111133u,
  0xd26969bbu, 0xa9d9d970u, 0x078e8e89u, 0x339494a7u,
  0x2d9b9bb6u, 0x3c1e1e22u, 0x15878792u, 0xc9e9e920u,
  0x87cece49u, 0xaa5555ffu, 0x50282878u, 0xa5dfdf7au,
  0x038c8c8fu, 0x59a1a1f8u, 0x09898980u, 0x1a0d0d17u,
  0x65bfbfdau, 0xd7e6e631u, 0x844242c6u, 0xd06868b8u,
  0x824141c3u, 0x299999b0u, 0x5a2d2d77u, 0x1e0f0f11u,
  0x7bb0b0cbu, 0xa85454fcu, 0x6dbbbbd6u, 0x2c16163au,
};
static const uint32_t TE1[256] =
{
  0xa5c66363u, 0x84f87c7cu, 0x99ee7777u, 0x8df67b7bu,
  0x0dfff2f2u, 0xbdd66b6bu, 0xb1de6f6fu, 0x5491c5c5u,
  0x50603030u, 0x03020101u, 0xa9ce6767u, 0x7d562b2bu,
  0x19e7fefeu, 0x62b5d7d7u, 0xe64dababu, 0x9aec7676u,
  0x458fcacau, 0x9d1f8282u, 0x4089c9c9u, 0x87fa7d7du,
  0x15effafau, 0xebb25959u, 0xc98e4747u, 0x0bfbf0f0u,
  0xec41adadu, 0x67b3d4d4u, 0xfd5fa2a2u, 0xea45afafu,
  0xbf239c9cu, 0xf753a4a4u, 0x96e47272u, 0x5b9bc0c0u,
  0xc275b7b7u, 0x1ce1fdfdu, 0xae3d9393u, 0x6a4c2626u,
  0x5a6c3636u, 0x417e3f3fu, 0x02f5f7f7u, 0x4f83ccccu,
  0x5c683434u, 0xf451a5a5u, 0x34d1e5e5u, 0x08f9f1f1u,
  0x93e27171u, 0x73abd8d8u, 0x53623131u, 0x3f2a1515u,
  0x0c080404u, 0x5295c7c7u, 0x65462323u, 0x5e9dc3c3u,
  0x28301818u, 0xa1379696u, 0x0f0a0505u, 0xb52f9a9au,
  0x090e0707u, 0x36241212u, 0x9b1b8080u, 0x3ddfe2e2u,
  0x26cdebebu, 0x694e2727u, 0xcd7fb2b2u, 0x9fea7575u,
  0x1b120909u, 0x9e1d8383u, 0x74582c2cu, 0x2e341a1au,
  0x2d361b1bu, 0xb2dc6e6eu, 0xeeb45a5au, 0xfb5ba0a0u,
  0xf6a45252u, 0x4d763b3bu, 0x61b7d6d6u, 0xce7db3b3u,
  0x7b522929u, 0x3edde3e3u, 0x715e2f2fu, 0x97138484u,
  0xf5a65353u, 0x68b9d1d1u, 0x00000000u, 0x2cc1ededu,
  0x60402020u, 0x1fe3fcfcu, 0xc879b1b1u, 0xedb65b5bu,
  0xbed46a6au, 0x468dcbcbu, 0xd967bebeu, 0x4b723939u,
  0xde944a4au, 0xd4984c4cu, 0xe8b05858u, 0x4a85cfcfu,
  0x6bbbd0d0u, 0x2ac5efefu, 0xe54faaaau, 0x16edfbfbu,
  0xc5864343u, 0xd79a4d4du, 0x55663333u, 0x94118585u,
  0xcf8a4545u, 0x10e9f9f9u, 0x06040202u, 0x81fe7f7fu,
  0xf0a05050u, 0x44783c3cu, 0xba259f9fu, 0xe34ba8a8u,
  0xf3a25151u, 0xfe5da3a3u, 0xc0804040u, 0x8a058f8fu,
  0xad3f9292u, 0xbc219d9du, 0x48703838u, 0x04f1f5f5u,
  0xdf63bcbcu, 0xc177b6b6u, 0x75afdadau, 0x63422121u,
  0x30201010u, 0x1ae5ffffu, 0x0efdf3f3u, 0x6dbfd2d2u,
  0x4c81cdcdu, 0x14180c0cu, 0x35261313u, 0x2fc3ececu,
  0xe1be5f5fu, 0xa2359797u, 0xcc884444u, 0x392e1717u,
  0x5793c4c4u, 0xf255a7a7u, 0x82fc7e7eu, 0x477a3d3du,
  0xacc86464u, 0xe7ba5d5du, 0x2b321919u, 0x95e67373u,
  0xa0c06060u, 0x98198181u, 0xd19e4f4fu, 0x7fa3dcdcu,
  0x66442222u, 0x7e542a2au, 0xab3b9090u, 0x830b8888u,
  0xca8c4646u, 0x29c7eeeeu, 0xd36bb8b8u, 0x3c281414u,
  0x79a7dedeu, 0xe2bc5e5eu, 0x1d160b0bu, 0x76addbdbu,
  0x3bdbe0e0u, 0x56643232u, 0x4e743a3au, 0x1e140a0au,
  0xdb924949u, 0x0a0c0606u, 0x6c482424u, 0xe4b85c5cu,
  0x5d9fc2c2u, 0x6ebdd3d3u, 0xef43acacu, 0xa6c46262u,
  0xa8399191u, 0xa4319595u, 0x37d3e4e4u, 0x8bf27979u,
  0x32d5e7e7u, 0x438bc8c8u, 0x596e3737u, 0xb7da6d6du,
  0x8c018d8du, 0x64b1d5d5u, 0xd29c4e4eu, 0xe049a9a9u,
  0xb4d86c6cu, 0xfaac5656u, 0x07f3f4f4u, 0x25cfeaeau,
  0xafca6565u, 0x8ef47a7au, 0xe947aeaeu, 0x18100808u,
  0xd56fbabau, 0x88f07878u, 0x6f4a2525u, 0x725c2e2eu,
  0x24381c1cu, 0xf157a6a6u, 0xc773b4b4u, 0x5197c6c6u,
  0x23cbe8e8u, 0x7ca1ddddu, 0x9ce87474u, 0x213e1f1fu,
  0xdd964b4bu, 0xdc61bdbdu, 0x860d8b8bu, 0x850f8a8au,
  0x90e07070u, 0x427c3e3eu, 0xc471b5b5u, 0xaacc6666u,
  0xd8904848u, 0x05060303u, 0x01f7f6f6u, 0x121c0e0eu,
  0xa3c26161u, 0x5f6a3535u, 0xf9ae5757u, 0xd069b9b9u,
  0x91178686u, 0x5899c1c1u, 0x273a1d1du, 0xb9279e9eu,
  0x38d9e1e1u, 0x13ebf8f8u, 0xb32b9898u, 0x33221111u,
  0xbbd26969u, 0x70a9d9d9u, 0x89078e8eu, 0xa7339494u,
  0xb62d9b9bu, 0x223c1e1eu, 0x92158787u, 0x20c9e9e9u,
  0x4987ceceu, 0xffaa5555u, 0x78502828u, 0x7aa5dfdfu,
  0x8f038c8cu, 0xf859a1a1u, 0x80098989u, 0x171a0d0du,
  0xda65bfbfu, 0x31d7e6e6u, 0xc6844242u, 0xb8d06868u,
  0xc3824141u, 0xb0299999u, 0x775a2d2du, 0x111e0f0fu,
  0xcb7bb0b0u, 0xfca85454u, 0xd66dbbbbu, 0x3a2c1616u,
};
static const uint32_t TE2[256] =
{
  0x63a5c663u, 0x7c84f87cu, 0x7799ee77u, 0x7b8df67bu,
  0xf20dfff2u, 0x6bbdd66bu, 0x6fb1de6fu, 0xc55491c5u,
  0x30506030u, 0x01030201u, 0x67a9ce67u, 0x2b7d562bu,
  0xfe19e7feu, 0xd762b5d7u, 0xabe64dabu, 0x769aec76u,
  0xca458fcau, 0x829d1f82u, 0xc94089c9u, 0x7d87fa7du,
  0xfa15effau, 0x59ebb259u, 0x47c98e47u, 0xf00bfbf0u,
  0xadec41adu, 0xd467b3d4u, 0xa2fd5fa2u, 0xafea45afu,
  0x9cbf239cu, 0xa4f753a4u, 0x7296e472u, 0xc05b9bc0u,
  0xb7c275b7u, 0xfd1ce1fdu, 0x93ae3d93u, 0x266a4c26u,
  0x365a6c36u, 0x3f417e3fu, 0xf702f5f7u, 0xcc4f83ccu,
  0x345c6834u, 0xa5f451a5u, 0xe534d1e5u, 0xf108f9f1u,
  0x7193e271u, 0xd873abd8u, 0x31536231u, 0x153f2a15u,
  0x040c0804u, 0xc75295c7u, 0x23654623u, 0xc35e9dc3u,
  0x18283018u, 0x96a13796u, 0x050f0a05u, 0x9ab52f9au,
  0x07090e07u, 0x12362412u, 0x809b1b80u, 0xe23ddfe2u,
  0xeb26cdebu, 0x27694e27u, 0xb2cd7fb2u, 0x759fea75u,
  0x091b1209u, 0x839e1d83u, 0x2c74582cu, 0x1a2e341au,
  0x1b2d361bu, 0x6eb2dc6eu, 0x5aeeb45au, 0xa0fb5ba0u,
  0x52f6a452u, 0x3b4d763bu, 0xd661b7d6u, 0xb3ce7db3u,
  0x297b5229u, 0xe33edde3u, 0x2f715e2fu, 0x84971384u,
  0x53f5a653u, 0xd168b9d1u, 0x00000000u, 0xed2cc1edu,
  0x20604020u, 0xfc1fe3fcu, 0xb1c879b1u, 0x5bedb65bu,
  0x6abed46au, 0xcb468dcbu, 0xbed967beu, 0x394b7239u,
  0x4ade944au, 0x4cd4984cu, 0x58e8b058u, 0xcf4a85cfu,
  0xd06bbbd0u, 0xef2ac5efu, 0xaae54faau, 0xfb16edfbu,
  0x43c58643u, 0x4dd79a4du, 0x33556633u, 0x85941185u,
  0x45cf8a45u, 0xf910e9f9u, 0x02060402u, 0x7f81fe7fu,
  0x50f0a050u, 0x3c44783cu, 0x9fba259fu, 0xa8e34ba8u,
  0x51f3a251u, 0xa3fe5da3u, 0x40c08040u, 0x8f8a058fu,
  0x92ad3f92u, 0x9dbc219du, 0x38487038u, 0xf504f1f5u,
  0xbcdf63bcu, 0xb6c177b6u, 0xda75afdau, 0x21634221u,
  0x10302010u, 0xff1ae5ffu, 0xf30efdf3u, 0xd26dbfd2u,
  0xcd4c81cdu, 0x0c14180cu, 0x13352613u, 0xec2fc3ecu,
  0x5fe1be5fu, 0x97a23597u, 0x44cc8844u, 0x17392e17u,
  0xc45793c4u, 0xa7f255a7u, 0x7e82fc7eu, 0x3d477a3du,
  0x64acc864u, 0x5de7ba5du, 0x192b3219u, 0x7395e673u,
  0x60a0c060u, 0x81981981u, 0x4fd19e4fu, 0xdc7fa3dcu,
  0x22664422u, 0x2a7e542au, 0x90ab3b90u, 0x88830b88u,
  0x46ca8c46u, 0xee29c7eeu, 0xb8d36bb8u, 0x143c2814u,
  0xde79a7deu, 0x5ee2bc5eu, 0x0b1d160bu, 0xdb76addbu,
  0xe03bdbe0u, 0x32566432u, 0x3a4e743au, 0x0a1e140au,
  0x49db9249u, 0x060a0c06u, 0x246c4824u, 0x5ce4b85cu,
  0xc25d9fc2u, 0xd36ebdd3u, 0xacef43acu, 0x62a6c462u,
  0x91a83991u, 0x95a43195u, 0xe437d3e4u, 0x798bf279u,
  0xe732d5e7u, 0xc8438bc8u, 0x37596e37u, 0x6db7da6du,
  0x8d8c018du, 0xd564b1d5u, 0x4ed29c4eu, 0xa9e049a9u,
  0x6cb4d86cu, 0x56faac56u, 0xf407f3f4u, 0xea25cfeau,
  0x65afca65u, 0x7a8ef47au, 0xaee947aeu, 0x08181008u,
  0xbad56fbau, 0x7888f078u, 0x256f4a25u, 0x2e725c2eu,
  0x1c24381cu, 0xa6f157a6u, 0xb4c773b4u, 0xc65197c6u,
  0xe823cbe8u, 0xdd7ca1ddu, 0x749ce874u, 0x1f213e1fu,
  0x4bdd964bu, 0xbddc61bdu, 0x8b860d8bu, 0x8a850f8au,
  0x7090e070u, 0x3e427c3eu, 0xb5c471b5u, 0x66aacc66u,
  0x48d89048u, 0x03050603u, 0xf601f7f6u, 0x0e121c0eu,
  0x61a3c261u, 0x355f6a35u, 0x57f9ae57u, 0xb9d069b9u,
  0x86911786u, 0xc15899c1u, 0x1d273a1du, 0x9eb9279eu,
  0xe138d9e1u, 0xf813ebf8u, 0x98b32b98u, 0x11332211u,
  0x69bbd269u, 0xd970a9d9u, 0x8e89078eu, 0x94a73394u,
  0x9bb62d9bu, 0x1e223c1eu, 0x87921587u, 0xe920c9e9u,
  0xce4987ceu, 0x55ffaa55u, 0x28785028u, 0xdf7aa5dfu,
  0x8c8f038cu, 0xa1f859a1u, 0x89800989u, 0x0d171a0du,
  0xbfda65bfu, 0xe631d7e6u, 0x42c68442u, 0x68b8d068u,
  0x41c38241u, 0x99b02999u, 0x2d775a2du, 0x0f111e0fu,
  0xb0cb7bb0u, 0x54fca854u, 0xbbd66dbbu, 0x163a2c16u,
};
static const uint32_t TE3[256] =
{
  0x6363a5c6u, 0x7c7c84f8u, 0x777799eeu, 0x7b7b8df6u,
  0xf2f20dffu, 0x6b6bbdd6u, 0x6f6fb1deu, 0xc5c55491u,
  0x30305060u, 0x01010302u, 0x6767a9ceu, 0x2b2b7d56u,
  0xfefe19e7u, 0xd7d762b5u, 0xababe64du, 0x76769aecu,
  0xcaca458fu, 0x82829d1fu, 0xc9c94089u, 0x7d7d87fau,
  0xfafa15efu, 0x5959ebb2u, 0x4747c98eu, 0xf0f00bfbu,
  0xadadec41u, 0xd4d467b3u, 0xa2a2fd5fu, 0xafafea45u,
  0x9c9cbf23u, 0xa4a4f753u, 0x727296e4u, 0xc0c05b9bu,
  0xb7b7c275u, 0xfdfd1ce1u, 0x9393ae3du, 0x26266a4cu,
  0x36365a6cu, 0x3f3f417eu, 0xf7f702f5u, 0xcccc4f83u,
  0x34345c68u, 0xa5a5f451u, 0xe5e534d1u, 0xf1f108f9u,
  0x717193e2u, 0xd8d873abu, 0x31315362u, 0x15153f2au,
  0x04040c08u, 0xc7c75295u, 0x23236546u, 0xc3c35e9du,
  0x18182830u, 0x9696a137u, 0x05050f0au, 0x9a9ab52fu,
  0x0707090eu, 0x12123624u, 0x80809b1bu, 0xe2e23ddfu,
  0xebeb26cdu, 0x2727694eu, 0xb2b2cd7fu, 0x75759feau,
  0x09091b12u, 0x83839e1du, 0x2c2c7458u, 0x1a1a2e34u,
  0x1b1b2d36u, 0x6e6eb2dcu, 0x5a5aeeb4u, 0xa0a0fb5bu,
  0x5252f6a4u, 0x3b3b4d76u, 0xd6d661b7u, 0xb3b3ce7du,
  0x29297b52u, 0xe3e33eddu, 0x2f2f715eu, 0x84849713u,
  0x5353f5a6u, 0xd1d168b9u, 0x00000000u, 0xeded2cc1u,
  0x20206040u, 0xfcfc1fe3u, 0xb1b1c879u, 0x5b5bedb6u,
  0x6a6abed4u, 0xcbcb468du, 0xbebed967u, 0x39394b72u,
  0x4a4ade94u, 0x4c4cd498u, 0x5858e8b0u, 0xcfcf4a85u,
  0xd0d06bbbu, 0xefef2ac5u, 0xaaaae54fu, 0xfbfb16edu,
  0x4343c586u, 0x4d4dd79au, 0x33335566u, 0x85859411u,
  0x4545cf8au, 0xf9f910e9u, 0x02020604u, 0x7f7f81feu,
  0x5050f0a0u, 0x3c3c4478u, 0x9f9fba25u, 0xa8a8e34bu,
  0x5151f3a2u, 0xa3a3fe5du, 0x4040c080u, 0x8f8f8a05u,
  0x9292ad3fu, 0x9d9dbc21u, 0x38384870u, 0xf5f504f1u,
  0xbcbcdf63u, 0xb6b6c177u, 0xdada75afu, 0x21216342u,
  0x10103020u, 0xffff1ae5u, 0xf3f30efdu, 0xd2d26dbfu,
  0xcdcd4c81u, 0x0c0c1418u, 0x13133526u, 0xecec2fc3u,
  0x5f5fe1beu, 0x9797a235u, 0x4444cc88u, 0x1717392eu,
  0xc4c45793u, 0xa7a7f255u, 0x7e7e82fcu, 0x3d3d477au,
  0x6464acc8u, 0x5d5de7bau, 0x19192b32u, 0x737395e6u,
  0x6060a0c0u, 0x81819819u, 0x4f4fd19eu, 0xdcdc7fa3u,
  0x22226644u, 0x2a2a7e54u, 0x9090ab3bu, 0x8888830bu,
  0x4646ca8cu, 0xeeee29c7u, 0xb8b8d36bu, 0x14143c28u,
  0xdede79a7u, 0x5e5ee2bcu, 0x0b0b1d16u, 0xdbdb76adu,
  0xe0e03bdbu, 0x32325664u, 0x3a3a4e74u, 0x0a0a1e14u,
  0x4949db92u, 0x06060a0cu, 0x24246c48u, 0x5c5ce4b8u,
  0xc2c25d9fu, 0xd3d36ebdu, 0xacacef43u, 0x6262a6c4u,
  0x9191a839u, 0x9595a431u, 0xe4e437d3u, 0x79798bf2u,
  0xe7e732d5u, 0xc8c8438bu, 0x3737596eu, 0x6d6db7dau,
  0x8d8d8c01u, 0xd5d564b1u, 0x4e4ed29cu, 0xa9a9e049u,
  0x6c6cb4d8u, 0x5656faacu, 0xf4f407f3u, 0xeaea25cfu,
  0x6565afcau, 0x7a7a8ef4u, 0xaeaee947u, 0x08081810u,
  0xbabad56fu, 0x787888f0u, 0x25256f4au, 0x2e2e725cu,
  0x1c1c2438u, 0xa6a6f157u, 0xb4b4c773u, 0xc6c65197u,
  0xe8e823cbu, 0xdddd7ca1u, 0x74749ce8u, 0x1f1f213eu,
  0x4b4bdd96u, 0xbdbddc61u, 0x8b8b860du, 0x8a8a850fu,
  0x707090e0u, 0x3e3e427cu, 0xb5b5c471u, 0x6666aaccu,
  0x4848d890u, 0x03030506u, 0xf6f601f7u, 0x0e0e121cu,
  0x6161a3c2u, 0x35355f6au, 0x5757f9aeu, 0xb9b9d069u,
  0x86869117u, 0xc1c15899u, 0x1d1d273au, 0x9e9eb927u,
  0xe1e138d9u, 0xf8f813ebu, 0x9898b32bu, 0x11113322u,
  0x6969bbd2u, 0xd9d970a9u, 0x8e8e8907u, 0x9494a733u,
  0x9b9bb62du, 0x1e1e223cu, 0x87879215u, 0xe9e920c9u,
  0xcece4987u, 0x5555ffaau, 0x28287850u, 0xdfdf7aa5u,
  0x8c8c8f03u, 0xa1a1f859u, 0x89898009u, 0x0d0d171au,
  0xbfbfda65u, 0xe6e631d7u, 0x4242c684u, 0x6868b8d0u,
  0x4141c382u, 0x9999b029u, 0x2d2d775au, 0x0f0f111eu,
  0xb0b0cb7bu, 0x5454fca8u, 0xbbbbd66du, 0x16163a2cu,
};
static const uint32_t TD0[256] =
{
  0x51f4a750u, 0x7e416553u, 0x1a17a4c3u, 0x3a275e96u,
  0x3bab6bcbu, 0x1f9d45f1u, 0xacfa58abu, 0x4be30393u,
  0x2030fa55u, 0xad766df6u, 0x88cc7691u, 0xf5024c25u,
  0x4fe5d7fcu, 0xc52acbd7u, 0x26354480u, 0xb562a38fu,
  0xdeb15a49u, 0x25ba1b67u, 0x45ea0e98u, 0x5dfec0e1u,
  0xc32f7502u, 0x814cf012u, 0x8d4697a3u, 0x6bd3f9c6u,
  0x038f5fe7u, 0x15929c95u, 0xbf6d7aebu, 0x955259dau,
  0xd4be832du, 0x587421d3u, 0x49e06929u, 0x8ec9c844u,
  0x75c2896au, 0xf48e7978u, 0x99583e6bu, 0x27b971ddu,
  0xbee14fb6u, 0xf088ad17u, 0xc920ac66u, 0x7dce3ab4u,
  0x63df4a18u, 0xe51a3182u, 0x97513360u, 0x62537f45u,
  0xb16477e0u, 0xbb6bae84u, 0xfe81a01cu, 0xf9082b94u,
  0x70486858u, 0x8f45fd19u, 0x94de6c87u, 0x527bf8b7u,
  0xab73d323u, 0x724b02e2u, 0xe31f8f57u, 0x6655ab2au,
  0xb2eb2807u, 0x2fb5c203u, 0x86c57b9au, 0xd33708a5u,
  0x302887f2u, 0x23bfa5b2u, 0x02036abau, 0xed16825cu,
  0x8acf1c2bu, 0xa779b492u, 0xf307f2f0u, 0x4e69e2a1u,
  0x65daf4cdu, 0x0605bed5u, 0xd134621fu, 0xc4a6fe8au,
  0x342e539du, 0xa2f355a0u, 0x058ae132u, 0xa4f6eb75u,
  0x0b83ec39u, 0x4060efaau, 0x5e719f06u, 0xbd6e1051u,
  0x3e218af9u, 0x96dd063du, 0xdd3e05aeu, 0x4de6bd46u,
  0x91548db5u, 0x71c45d05u, 0x0406d46fu, 0x605015ffu,
  0x1998fb24u, 0xd6bde997u, 0x894043ccu, 0x67d99e77u,
  0xb0e842bdu, 0x07898b88u, 0xe7195b38u, 0x79c8eedbu,
  0xa17c0a47u, 0x7c420fe9u, 0xf8841ec9u, 0x00000000u,
  0x09808683u, 0x322bed48u, 0x1e1170acu, 0x6c5a724eu,
  0xfd0efffbu, 0x0f853856u, 0x3daed51eu, 0x362d3927u,
  0x0a0fd964u, 0x685ca621u, 0x9b5b54d1u, 0x24362e3au,
  0x0c0a67b1u, 0x9357e70fu, 0xb4ee96d2u, 0x1b9b919eu,
  0x80c0c54fu, 0x61dc20a2u, 0x5a774b69u, 0x1c121a16u,
  0xe293ba0au, 0xc0a02ae5u, 0x3c22e043u, 0x121b171du,
  0x0e090d0bu, 0xf28bc7adu, 0x2db6a8b9u, 0x141ea9c8u,
  0x57f11985u, 0xaf75074cu, 0xee99ddbbu, 0xa37f60fdu,
  0xf701269fu, 0x5c72f5bcu, 0x44663bc5u, 0x5bfb7e34u,
  0x8b432976u, 0xcb23c6dcu, 0xb6edfc68u, 0xb8e4f163u,
  0xd731dccau, 0x42638510u, 0x13972240u, 0x84c61120u,
  0x854a247du, 0xd2bb3df8u, 0xaef93211u, 0xc729a16du,
  0x1d9e2f4bu, 0xdcb230f3u, 0x0d8652ecu, 0x77c1e3d0u,
  0x2bb3166cu, 0xa970b999u, 0x119448fau, 0x47e96422u,
  0xa8fc8cc4u, 0xa0f03f1au, 0x567d2cd8u, 0x223390efu,
  0x87494ec7u, 0xd938d1c1u, 0x8ccaa2feu, 0x98d40b36u,
  0xa6f581cfu, 0xa57ade28u, 0xdab78e26u, 0x3fadbfa4u,
  0x2c3a9de4u, 0x5078920du, 0x6a5fcc9bu, 0x547e4662u,
  0xf68d13c2u, 0x90d8b8e8u, 0x2e39f75eu, 0x82c3aff5u,
  0x9f5d80beu, 0x69d0937cu, 0x6fd52da9u, 0xcf2512b3u,
  0xc8ac993bu, 0x10187da7u, 0xe89c636eu, 0xdb3bbb7bu,
  0xcd267809u, 0x6e5918f4u, 0xec9ab701u, 0x834f9aa8u,
  0xe6956e65u, 0xaaffe67eu, 0x21bccf08u, 0xef15e8e6u,
  0xbae79bd9u, 0x4a6f36ceu, 0xea9f09d4u, 0x29b07cd6u,
  0x31a4b2afu, 0x2a3f2331u, 0xc6a59430u, 0x35a266c0u,
  0x744ebc37u, 0xfc82caa6u, 0xe090d0b0u, 0x33a7d815u,
  0xf104984au, 0x41ecdaf7u, 0x7fcd500eu, 0x1791f62fu,
  0x764dd68du, 0x43efb04du, 0xccaa4d54u, 0xe49604dfu,
  0x9ed1b5e3u, 0x4c6a881bu, 0xc12c1fb8u, 0x4665517fu,
  0x9d5eea04u, 0x018c355du, 0xfa877473u, 0xfb0b412eu,
  0xb3671d5au, 0x92dbd252u, 0xe9105633u, 0x6dd64713u,
  0x9ad7618cu, 0x37a10c7au, 0x59f8148eu, 0xeb133c89u,
  0xcea927eeu, 0xb761c935u, 0xe11ce5edu, 0x7a47b13cu,
  0x9cd2df59u, 0x55f2733fu, 0x1814ce79u, 0x73c737bfu,
  0x53f7cdeau, 0x5ffdaa5bu, 0xdf3d6f14u, 0x7844db86u,
  0xcaaff381u, 0xb968c43eu, 0x3824342cu, 0xc2a3405fu,
  0x161dc372u, 0xbce2250cu, 0x283c498bu, 0xff0d9541u,
  0x39a80171u, 0x080cb3deu, 0xd8b4e49cu, 0x6456c190u,
  0x7bcb8461u, 0xd532b670u, 0x486c5c74u, 0xd0b85742u,
};
static const uint32_t TD1[256] =
{
  0x5051f4a7u, 0x537e4165u, 0xc31a17a4u, 0x963a275eu,
  0xcb3bab6bu, 0xf11f9d45u, 0xabacfa58u, 0x934be303u,
  0x552030fau, 0xf6ad766du, 0x9188cc76u, 0x25f5024cu,
  0xfc4fe5d7u, 0xd7c52acbu, 0x80263544u, 0x8fb562a3u,
  0x49deb15au, 0x6725ba1bu, 0x9845ea0eu, 0xe15dfec0u,
  0x02c32f75u, 0x12814cf0u, 0xa38d4697u, 0xc66bd3f9u,
  0xe7038f5fu, 0x9515929cu, 0xebbf6d7au, 0xda955259u,
  0x2dd4be83u, 0xd3587421u, 0x2949e069u, 0x448ec9c8u,
  0x6a75c289u, 0x78f48e79u, 0x6b99583eu, 0xdd27b971u,
  0xb6bee14fu, 0x17f088adu, 0x66c920acu, 0xb47dce3au,
  0x1863df4au, 0x82e51a31u, 0x60975133u, 0x4562537fu,
  0xe0b16477u, 0x84bb6baeu, 0x1cfe81a0u, 0x94f9082bu,
  0x58704868u, 0x198f45fdu, 0x8794de6cu, 0xb7527bf8u,
  0x23ab73d3u, 0xe2724b02u, 0x57e31f8fu, 0x2a6655abu,
  0x07b2eb28u, 0x032fb5c2u, 0x9a86c57bu, 0xa5d33708u,
  0xf2302887u, 0xb223bfa5u, 0xba02036au, 0x5ced1682u,
  0x2b8acf1cu, 0x92a779b4u, 0xf0f307f2u, 0xa14e69e2u,
  0xcd65daf4u, 0xd50605beu, 0x1fd13462u, 0x8ac4a6feu,
  0x9d342e53u, 0xa0a2f355u, 0x32058ae1u, 0x75a4f6ebu,
  0x390b83ecu, 0xaa4060efu, 0x065e719fu, 0x51bd6e10u,
  0xf93e218au, 0x3d96dd06u, 0xaedd3e05u, 0x464de6bdu,
  0xb591548du, 0x0571c45du, 0x6f0406d4u, 0xff605015u,
  0x241998fbu, 0x97d6bde9u, 0xcc894043u, 0x7767d99eu,
  0xbdb0e842u, 0x8807898bu, 0x38e7195bu, 0xdb79c8eeu,
  0x47a17c0au, 0xe97c420fu, 0xc9f8841eu, 0x00000000u,
  0x83098086u, 0x48322bedu, 0xac1e1170u, 0x4e6c5a72u,
  0xfbfd0effu, 0x560f8538u, 0x1e3daed5u, 0x27362d39u,
  0x640a0fd9u, 0x21685ca6u, 0xd19b5b54u, 0x3a24362eu,
  0xb10c0a67u, 0x0f9357e7u, 0xd2b4ee96u, 0x9e1b9b91u,
  0x4f80c0c5u, 0xa261dc20u, 0x695a774bu, 0x161c121au,
  0x0ae293bau, 0xe5c0a02au, 0x433c22e0u, 0x1d121b17u,
  0x0b0e090du, 0xadf28bc7u, 0xb92db6a8u, 0xc8141ea9u,
  0x8557f119u, 0x4caf7507u, 0xbbee99ddu, 0xfda37f60u,
  0x9ff70126u, 0xbc5c72f5u, 0xc544663bu, 0x345bfb7eu,
  0x768b4329u, 0xdccb23c6u, 0x68b6edfcu, 0x63b8e4f1u,
  0xcad731dcu, 0x10426385u, 0x40139722u, 0x2084c611u,
  0x7d854a24u, 0xf8d2bb3du, 0x11aef932u, 0x6dc729a1u,
  0x4b1d9e2fu, 0xf3dcb230u, 0xec0d8652u, 0xd077c1e3u,
  0x6c2bb316u, 0x99a970b9u, 0xfa119448u, 0x2247e964u,
  0xc4a8fc8cu, 0x1aa0f03fu, 0xd8567d2cu, 0xef223390u,
  0xc787494eu, 0xc1d938d1u, 0xfe8ccaa2u, 0x3698d40bu,
  0xcfa6f581u, 0x28a57adeu, 0x26dab78eu, 0xa43fadbfu,
  0xe42c3a9du, 0x0d507892u, 0x9b6a5fccu, 0x62547e46u,
  0xc2f68d13u, 0xe890d8b8u, 0x5e2e39f7u, 0xf582c3afu,
  0xbe9f5d80u, 0x7c69d093u, 0xa96fd52du, 0xb3cf2512u,
  0x3bc8ac99u, 0xa710187du, 0x6ee89c63u, 0x7bdb3bbbu,
  0x09cd2678u, 0xf46e5918u, 0x01ec9ab7u, 0xa8834f9au,
  0x65e6956eu, 0x7eaaffe6u, 0x0821bccfu, 0xe6ef15e8u,
  0xd9bae79bu, 0xce4a6f36u, 0xd4ea9f09u, 0xd629b07cu,
  0xaf31a4b2u, 0x312a3f23u, 0x30c6a594u, 0xc035a266u,
  0x37744ebcu, 0xa6fc82cau, 0xb0e090d0u, 0x1533a7d8u,
  0x4af10498u, 0xf741ecdau, 0x0e7fcd50u, 0x2f1791f6u,
  0x8d764dd6u, 0x4d43efb0u, 0x54ccaa4du, 0xdfe49604u,
  0xe39ed1b5u, 0x1b4c6a88u, 0xb8c12c1fu, 0x7f466551u,
  0x049d5eeau, 0x5d018c35u, 0x73fa8774u, 0x2efb0b41u,
  0x5ab3671du, 0x5292dbd2u, 0x33e91056u, 0x136dd647u,
  0x8c9ad761u, 0x7a37a10cu, 0x8e59f814u, 0x89eb133cu,
  0xeecea927u, 0x35b761c9u, 0xede11ce5u, 0x3c7a47b1u,
  0x599cd2dfu, 0x3f55f273u, 0x791814ceu, 0xbf73c737u,
  0xea53f7cdu, 0x5b5ffdaau, 0x14df3d6fu, 0x867844dbu,
  0x81caaff3u, 0x3eb968c4u, 0x2c382434u, 0x5fc2a340u,
  0x72161dc3u, 0x0cbce225u, 0x8b283c49u, 0x41ff0d95u,
  0x7139a801u, 0xde080cb3u, 0x9cd8b4e4u, 0x906456c1u,
  0x617bcb84u, 0x70d532b6u, 0x74486c5cu, 0x42d0b857u,
};
static const uint32_t TD2[256] =
{
  0xa75051f4u, 0x65537e41u, 0xa4c31a17u, 0x5e963a27u,
  0x6bcb3babu, 0x45f11f9du, 0x58abacfau, 0x03934be3u,
  0xfa552030u, 0x6df6ad76u, 0x769188ccu, 0x4c25f502u,
  0xd7fc4fe5u, 0xcbd7c52au, 0x44802635u, 0xa38fb562u,
  0x5a49deb1u, 0x1b6725bau, 0x0e9845eau, 0xc0e15dfeu,
  0x7502c32fu, 0xf012814cu, 0x97a38d46u, 0xf9c66bd3u,
  0x5fe7038fu, 0x9c951592u, 0x7aebbf6du, 0x59da9552u,
  0x832dd4beu, 0x21d35874u, 0x692949e0u, 0xc8448ec9u,
  0x896a75c2u, 0x7978f48eu, 0x3e6b9958u, 0x71dd27b9u,
  0x4fb6bee1u, 0xad17f088u, 0xac66c920u, 0x3ab47dceu,
  0x4a1863dfu, 0x3182e51au, 0x33609751u, 0x7f456253u,
  0x77e0b164u, 0xae84bb6bu, 0xa01cfe81u, 0x2b94f908u,
  0x68587048u, 0xfd198f45u, 0x6c8794deu, 0xf8b7527bu,
  0xd323ab73u, 0x02e2724bu, 0x8f57e31fu, 0xab2a6655u,
  0x2807b2ebu, 0xc2032fb5u, 0x7b9a86c5u, 0x08a5d337u,
  0x87f23028u, 0xa5b223bfu, 0x6aba0203u, 0x825ced16u,
  0x1c2b8acfu, 0xb492a779u, 0xf2f0f307u, 0xe2a14e69u,
  0xf4cd65dau, 0xbed50605u, 0x621fd134u, 0xfe8ac4a6u,
  0x539d342eu, 0x55a0a2f3u, 0xe132058au, 0xeb75a4f6u,
  0xec390b83u, 0xefaa4060u, 0x9f065e71u, 0x1051bd6eu,
  0x8af93e21u, 0x063d96ddu, 0x05aedd3eu, 0xbd464de6u,
  0x8db59154u, 0x5d0571c4u, 0xd46f0406u, 0x15ff6050u,
  0xfb241998u, 0xe997d6bdu, 0x43cc8940u, 0x9e7767d9u,
  0x42bdb0e8u, 0x8b880789u, 0x5b38e719u, 0xeedb79c8u,
  0x0a47a17cu, 0x0fe97c42u, 0x1ec9f884u, 0x00000000u,
  0x86830980u, 0xed48322bu, 0x70ac1e11u, 0x724e6c5au,
  0xfffbfd0eu, 0x38560f85u, 0xd51e3daeu, 0x3927362du,
  0xd9640a0fu, 0xa621685cu, 0x54d19b5bu, 0x2e3a2436u,
  0x67b10c0au, 0xe70f9357u, 0x96d2b4eeu, 0x919e1b9bu,
  0xc54f80c0u, 0x20a261dcu, 0x4b695a77u, 0x1a161c12u,
  0xba0ae293u, 0x2ae5c0a0u, 0xe0433c22u, 0x171d121bu,
  0x0d0b0e09u, 0xc7adf28bu, 0xa8b92db6u, 0xa9c8141eu,
  0x198557f1u, 0x074caf75u, 0xddbbee99u, 0x60fda37fu,
  0x269ff701u, 0xf5bc5c72u, 0x3bc54466u, 0x7e345bfbu,
  0x29768b43u, 0xc6dccb23u, 0xfc68b6edu, 0xf163b8e4u,
  0xdccad731u, 0x85104263u, 0x22401397u, 0x112084c6u,
  0x247d854au, 0x3df8d2bbu, 0x3211aef9u, 0xa16dc729u,
  0x2f4b1d9eu, 0x30f3dcb2u, 0x52ec0d86u, 0xe3d077c1u,
  0x166c2bb3u, 0xb999a970u, 0x48fa1194u, 0x642247e9u,
  0x8cc4a8fcu, 0x3f1aa0f0u, 0x2cd8567du, 0x90ef2233u,
  0x4ec78749u, 0xd1c1d938u, 0xa2fe8ccau, 0x0b3698d4u,
  0x81cfa6f5u, 0xde28a57au, 0x8e26dab7u, 0xbfa43fadu,
  0x9de42c3au, 0x920d5078u, 0xcc9b6a5fu, 0x4662547eu,
  0x13c2f68du, 0xb8e890d8u, 0xf75e2e39u, 0xaff582c3u,
  0x80be9f5du, 0x937c69d0u, 0x2da96fd5u, 0x12b3cf25u,
  0x993bc8acu, 0x7da71018u, 0x636ee89cu, 0xbb7bdb3bu,
  0x7809cd26u, 0x18f46e59u, 0xb701ec9au, 0x9aa8834fu,
  0x6e65e695u, 0xe67eaaffu, 0xcf0821bcu, 0xe8e6ef15u,
  0x9bd9bae7u, 0x36ce4a6fu, 0x09d4ea9fu, 0x7cd629b0u,
  0xb2af31a4u, 0x23312a3fu, 0x9430c6a5u, 0x66c035a2u,
  0xbc37744eu, 0xcaa6fc82u, 0xd0b0e090u, 0xd81533a7u,
  0x984af104u, 0xdaf741ecu, 0x500e7fcdu, 0xf62f1791u,
  0xd68d764du, 0xb04d43efu, 0x4d54ccaau, 0x04dfe496u,
  0xb5e39ed1u, 0x881b4c6au, 0x1fb8c12cu, 0x517f4665u,
  0xea049d5eu, 0x355d018cu, 0x7473fa87u, 0x412efb0bu,
  0x1d5ab367u, 0xd25292dbu, 0x5633e910u, 0x47136dd6u,
  0x618c9ad7u, 0x0c7a37a1u, 0x148e59f8u, 0x3c89eb13u,
  0x27eecea9u, 0xc935b761u, 0xe5ede11cu, 0xb13c7a47u,
  0xdf599cd2u, 0x733f55f2u, 0xce791814u, 0x37bf73c7u,
  0xcdea53f7u, 0xaa5b5ffdu, 0x6f14df3du, 0xdb867844u,
  0xf381caafu, 0xc43eb968u, 0x342c3824u, 0x405fc2a3u,
  0xc372161du, 0x250cbce2u, 0x498b283cu, 0x9541ff0du,
  0x017139a8u, 0xb3de080cu, 0xe49cd8b4u, 0xc1906456u,
  0x84617bcbu, 0xb670d532u, 0x5c74486cu, 0x5742d0b8u,
};
static const uint32_t TD3[256] =
{
  0xf4a75051u, 0x4165537eu, 0x17a4c31au, 0x275e963au,
  0xab6bcb3bu, 0x9d45f11fu, 0xfa58abacu, 0xe303934bu,
  0x30fa5520u, 0x766df6adu, 0xcc769188u, 0x024c25f5u,
  0xe5d7fc4fu, 0x2acbd7c5u, 0x35448026u, 0x62a38fb5u,
  0xb15a49deu, 0xba1b6725u, 0xea0e9845u, 0xfec0e15du,
  0x2f7502c3u, 0x4cf01281u, 0x4697a38du, 0xd3f9c66bu,
  0x8f5fe703u, 0x929c9515u, 0x6d7aebbfu, 0x5259da95u,
  0xbe832dd4u, 0x7421d358u, 0xe0692949u, 0xc9c8448eu,
  0xc2896a75u, 0x8e7978f4u, 0x583e6b99u, 0xb971dd27u,
  0xe14fb6beu, 0x88ad17f0u, 0x20ac66c9u, 0xce3ab47du,
  0xdf4a1863u, 0x1a3182e5u, 0x51336097u, 0x537f4562u,
  0x6477e0b1u, 0x6bae84bbu, 0x81a01cfeu, 0x082b94f9u,
  0x48685870u, 0x45fd198fu, 0xde6c8794u, 0x7bf8b752u,
  0x73d323abu, 0x4b02e272u, 0x1f8f57e3u, 0x55ab2a66u,
  0xeb2807b2u, 0xb5c2032fu, 0xc57b9a86u, 0x3708a5d3u,
  0x2887f230u, 0xbfa5b223u, 0x036aba02u, 0x16825cedu,
  0xcf1c2b8au, 0x79b492a7u, 0x07f2f0f3u, 0x69e2a14eu,
  0xdaf4cd65u, 0x05bed506u, 0x34621fd1u, 0xa6fe8ac4u,
  0x2e539d34u, 0xf355a0a2u, 0x8ae13205u, 0xf6eb75a4u,
  0x83ec390bu, 0x60efaa40u, 0x719f065eu, 0x6e1051bdu,
  0x218af93eu, 0xdd063d96u, 0x3e05aeddu, 0xe6bd464du,
  0x548db591u, 0xc45d0571u, 0x06d46f04u, 0x5015ff60u,
  0x98fb2419u, 0xbde997d6u, 0x4043cc89u, 0xd99e7767u,
  0xe842bdb0u, 0x898b8807u, 0x195b38e7u, 0xc8eedb79u,
  0x7c0a47a1u, 0x420fe97cu, 0x841ec9f8u, 0x00000000u,
  0x80868309u, 0x2bed4832u, 0x1170ac1eu, 0x5a724e6cu,
  0x0efffbfdu, 0x8538560fu, 0xaed51e3du, 0x2d392736u,
  0x0fd9640au, 0x5ca62168u, 0x5b54d19bu, 0x362e3a24u,
  0x0a67b10cu, 0x57e70f93u, 0xee96d2b4u, 0x9b919e1bu,
  0xc0c54f80u, 0xdc20a261u, 0x774b695au, 0x121a161cu,
  0x93ba0ae2u, 0xa02ae5c0u, 0x22e0433cu, 0x1b171d12u,
  0x090d0b0eu, 0x8bc7adf2u, 0xb6a8b92du, 0x1ea9c814u,
  0xf1198557u, 0x75074cafu, 0x99ddbbeeu, 0x7f60fda3u,
  0x01269ff7u, 0x72f5bc5cu, 0x663bc544u, 0xfb7e345bu,
  0x4329768bu, 0x23c6dccbu, 0xedfc68b6u, 0xe4f163b8u,
  0x31dccad7u, 0x63851042u, 0x97224013u, 0xc6112084u,
  0x4a247d85u, 0xbb3df8d2u, 0xf93211aeu, 0x29a16dc7u,
  0x9e2f4b1du, 0xb230f3dcu, 0x8652ec0du, 0xc1e3d077u,
  0xb3166c2bu, 0x70b999a9u, 0x9448fa11u, 0xe9642247u,
  0xfc8cc4a8u, 0xf03f1aa0u, 0x7d2cd856u, 0x3390ef22u,
  0x494ec787u, 0x38d1c1d9u, 0xcaa2fe8cu, 0xd40b3698u,
  0xf581cfa6u, 0x7ade28a5u, 0xb78e26dau, 0xadbfa43fu,
  0x3a9de42cu, 0x78920d50u, 0x5fcc9b6au, 0x7e466254u,
  0x8d13c2f6u, 0xd8b8e890u, 0x39f75e2eu, 0xc3aff582u,
  0x5d80be9fu, 0xd0937c69u, 0xd52da96fu, 0x2512b3cfu,
  0xac993bc8u, 0x187da710u, 0x9c636ee8u, 0x3bbb7bdbu,
  0x267809cdu, 0x5918f46eu, 0x9ab701ecu, 0x4f9aa883u,
  0x956e65e6u, 0xffe67eaau, 0xbccf0821u, 0x15e8e6efu,
  0xe79bd9bau, 0x6f36ce4au, 0x9f09d4eau, 0xb07cd629u,
  0xa4b2af31u, 0x3f23312au, 0xa59430c6u, 0xa266c035u,
  0x4ebc3774u, 0x82caa6fcu, 0x90d0b0e0u, 0xa7d81533u,
  0x04984af1u, 0xecdaf741u, 0xcd500e7fu, 0x91f62f17u,
  0x4dd68d76u, 0xefb04d43u, 0xaa4d54ccu, 0x9604dfe4u,
  0xd1b5e39eu, 0x6a881b4cu, 0x2c1fb8c1u, 0x65517f46u,
  0x5eea049du, 0x8c355d01u, 0x877473fau, 0x0b412efbu,
  0x671d5ab3u, 0xdbd25292u, 0x105633e9u, 0xd647136du,
  0xd7618c9au, 0xa10c7a37u, 0xf8148e59u, 0x133c89ebu,
  0xa927eeceu, 0x61c935b7u, 0x1ce5ede1u, 0x47b13c7au,
  0xd2df599cu, 0xf2733f55u, 0x14ce7918u, 0xc737bf73u,
  0xf7cdea53u, 0xfdaa5b5fu, 0x3d6f14dfu, 0x44db8678u,
  0xaff381cau, 0x68c43eb9u, 0x24342c38u, 0xa3405fc2u,
  0x1dc37216u, 0xe2250cbcu, 0x3c498b28u, 0x0d9541ffu,
  0xa8017139u, 0x0cb3de08u, 0xb4e49cd8u, 0x56c19064u,
  0xcb84617bu, 0x32b670d5u, 0x6c5c7448u, 0xb85742d0u,
};
static const uint8_t TD4[256] =
{
  0x52u, 0x09u, 0x6au, 0xd5u, 0x30u, 0x36u, 0xa5u, 0x38u,
  0xbfu, 0x40u, 0xa3u, 0x9eu, 0x81u, 0xf3u, 0xd7u, 0xfbu,
  0x7cu, 0xe3u, 0x39u, 0x82u, 0x9bu, 0x2fu, 0xffu, 0x87u,
  0x34u, 0x8eu, 0x43u, 0x44u, 0xc4u, 0xdeu, 0xe9u, 0xcbu,
  0x54u, 0x7bu, 0x94u, 0x32u, 0xa6u, 0xc2u, 0x23u, 0x3du,
  0xeeu, 0x4cu, 0x95u, 0x0bu, 0x42u, 0xfau, 0xc3u, 0x4eu,
  0x08u, 0x2eu, 0xa1u, 0x66u, 0x28u, 0xd9u, 0x24u, 0xb2u,
  0x76u, 0x5bu, 0xa2u, 0x49u, 0x6du, 0x8bu, 0xd1u, 0x25u,
  0x72u, 0xf8u, 0xf6u, 0x64u, 0x86u, 0x68u, 0x98u, 0x16u,
  0xd4u, 0xa4u, 0x5cu, 0xccu, 0x5du, 0x65u, 0xb6u, 0x92u,
  0x6cu, 0x70u, 0x48u, 0x50u, 0xfdu, 0xedu, 0xb9u, 0xdau,
  0x5eu, 0x15u, 0x46u, 0x57u, 0xa7u, 0x8du, 0x9du, 0x84u,
  0x90u, 0xd8u, 0xabu, 0x00u, 0x8cu, 0xbcu, 0xd3u, 0x0au,
  0xf7u, 0xe4u, 0x58u, 0x05u, 0xb8u, 0xb3u, 0x45u, 0x06u,
  0xd0u, 0x2cu, 0x1eu, 0x8fu, 0xcau, 0x3fu, 0x0fu, 0x02u,
  0xc1u, 0xafu, 0xbdu, 0x03u, 0x01u, 0x13u, 0x8au, 0x6bu,
  0x3au, 0x91u, 0x11u, 0x41u, 0x4fu, 0x67u, 0xdcu, 0xeau,
  0x97u, 0xf2u, 0xcfu, 0xceu, 0xf0u, 0xb4u, 0xe6u, 0x73u,
  0x96u, 0xacu, 0x74u, 0x22u, 0xe7u, 0xadu, 0x35u, 0x85u,
  0xe2u, 0xf9u, 0x37u, 0xe8u, 0x1cu, 0x75u, 0xdfu, 0x6eu,
  0x47u, 0xf1u, 0x1au, 0x71u, 0x1du, 0x29u, 0xc5u, 0x89u,
  0x6fu, 0xb7u, 0x62u, 0x0eu, 0xaau, 0x18u, 0xbeu, 0x1bu,
  0xfcu, 0x56u, 0x3eu, 0x4bu, 0xc6u, 0xd2u, 0x79u, 0x20u,
  0x9au, 0xdbu, 0xc0u, 0xfeu, 0x78u, 0xcdu, 0x5au, 0xf4u,
  0x1fu, 0xddu, 0xa8u, 0x33u, 0x88u, 0x07u, 0xc7u, 0x31u,
  0xb1u, 0x12u, 0x10u, 0x59u, 0x27u, 0x80u, 0xecu, 0x5fu,
  0x60u, 0x51u, 0x7fu, 0xa9u, 0x19u, 0xb5u, 0x4au, 0x0du,
  0x2du, 0xe5u, 0x7au, 0x9fu, 0x93u, 0xc9u, 0x9cu, 0xefu,
  0xa0u, 0xe0u, 0x3bu, 0x4du, 0xaeu, 0x2au, 0xf5u, 0xb0u,
  0xc8u, 0xebu, 0xbbu, 0x3cu, 0x83u, 0x53u, 0x99u, 0x61u,
  0x17u, 0x2bu, 0x04u, 0x7eu, 0xbau, 0x77u, 0xd6u, 0x26u,
  0xe1u, 0x69u, 0x14u, 0x63u, 0x55u, 0x21u, 0x0cu, 0x7du,
};
static const uint32_t rcon[] =
{
  0x01000000, 0x02000000, 0x04000000, 0x08000000,
  0x10000000, 0x20000000, 0x40000000, 0x80000000,
  0x1b000000, 0x36000000, /* for 128-bit blocks, Rijndael never uses more than 10 rcon values */
};

#define GETU32(pt) (((uint32_t)(pt)[0] << 24)  \
                   ^ ((uint32_t)(pt)[1] << 16) \
                   ^ ((uint32_t)(pt)[2] <<  8) \
                   ^ ((uint32_t)(pt)[3]))

#define PUTU32(ct, st) { \
                          (ct)[0] = (uint8_t)((st) >> 24);\
                          (ct)[1] = (uint8_t)((st) >> 16);\
                          (ct)[2] = (uint8_t)((st) >>  8);\
                          (ct)[3] = (uint8_t)(st); }

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Expand the cipher key into the encryption key schedule.
 * @return the number of rounds for the given cipher key size.
 */

int rijndael_key_setup_enc(FAR uint32_t *rk,
                           FAR const uint8_t *cipherkey,
                           int keybits)
{
  int i = 0;
  uint32_t temp;

  rk[0] = GETU32(cipherkey);
  rk[1] = GETU32(cipherkey + 4);
  rk[2] = GETU32(cipherkey + 8);
  rk[3] = GETU32(cipherkey + 12);
  if (keybits == 128)
    {
      for (; ; )
        {
          temp  = rk[3];
          rk[4] = rk[0] ^
            (TE2[(temp >> 16) & 0xff] & 0xff000000) ^
            (TE3[(temp >> 8) & 0xff] & 0x00ff0000) ^
            (TE0[(temp) & 0xff] & 0x0000ff00) ^
            (TE1[(temp >> 24)] & 0x000000ff) ^
            rcon[i];
          rk[5] = rk[1] ^ rk[4];
          rk[6] = rk[2] ^ rk[5];
          rk[7] = rk[3] ^ rk[6];
          if (++i == 10)
            {
              return 10;
            }

          rk += 4;
        }
    }

  rk[4] = GETU32(cipherkey + 16);
  rk[5] = GETU32(cipherkey + 20);
  if (keybits == 192)
    {
      for (; ; )
        {
          temp = rk[5];
          rk[6] = rk[0] ^
            (TE2[(temp >> 16) & 0xff] & 0xff000000) ^
            (TE3[(temp >> 8) & 0xff] & 0x00ff0000) ^
            (TE0[(temp) & 0xff] & 0x0000ff00) ^
            (TE1[(temp >> 24)] & 0x000000ff) ^
            rcon[i];
          rk[7] = rk[1] ^ rk[6];
          rk[8] = rk[2] ^ rk[7];
          rk[9] = rk[3] ^ rk[8];
          if (++i == 8)
            {
              return 12;
            }

          rk[10] = rk[4] ^ rk[9];
          rk[11] = rk[5] ^ rk[10];
          rk += 6;
        }
    }

  rk[6] = GETU32(cipherkey + 24);
  rk[7] = GETU32(cipherkey + 28);
  if (keybits == 256)
    {
      for (; ; )
        {
          temp = rk[7];
          rk[8] = rk[0] ^
            (TE2[(temp >> 16) & 0xff] & 0xff000000) ^
            (TE3[(temp >> 8) & 0xff] & 0x00ff0000) ^
            (TE0[(temp) & 0xff] & 0x0000ff00) ^
            (TE1[(temp >> 24)] & 0x000000ff) ^
            rcon[i];
          rk[9] = rk[1] ^ rk[8];
          rk[10] = rk[2] ^ rk[9];
          rk[11] = rk[3] ^ rk[10];
          if (++i == 7)
            {
              return 14;
            }

          temp = rk[11];
          rk[12] = rk[4] ^
            (TE2[(temp >> 24)] & 0xff000000) ^
            (TE3[(temp >> 16) & 0xff] & 0x00ff0000) ^
            (TE0[(temp >> 8) & 0xff] & 0x0000ff00) ^
            (TE1[(temp) & 0xff] & 0x000000ff);
          rk[13] = rk[5] ^ rk[12];
          rk[14] = rk[6] ^ rk[13];
              rk[15] = rk[7] ^ rk[14];
          rk += 8;
        }
    }

  return 0;
}

/* Expand the cipher key into the decryption key schedule.
 * @return the number of rounds for the given cipher key size.
 */

int rijndael_key_setup_dec(FAR uint32_t *rk,
                           FAR const uint8_t *cipherkey,
                           int keybits)
{
  int nr;
  int i;
  int j;
  uint32_t temp;

  /* expand the cipher key: */

  nr = rijndael_key_setup_enc(rk, cipherkey, keybits);

  /* invert the order of the round keys: */

  for (i = 0, j = 4 * nr; i < j; i += 4, j -= 4)
    {
      temp = rk[i];
      rk[i] = rk[j];
      rk[j] = temp;
      temp = rk[i + 1];
      rk[i + 1] = rk[j + 1];
      rk[j + 1] = temp;
      temp = rk[i + 2];
      rk[i + 2] = rk[j + 2];
      rk[j + 2] = temp;
      temp = rk[i + 3];
      rk[i + 3] = rk[j + 3];
      rk[j + 3] = temp;
    }

  /* apply the inverse MixColumn transform to
   * all round keys but the first and the last:
   */

  for (i = 1; i < nr; i++)
    {
      rk += 4;
      rk[0] =
        TD0[TE1[(rk[0] >> 24)] & 0xff] ^
        TD1[TE1[(rk[0] >> 16) & 0xff] & 0xff] ^
        TD2[TE1[(rk[0] >> 8) & 0xff] & 0xff] ^
        TD3[TE1[(rk[0]) & 0xff] & 0xff];
      rk[1] =
        TD0[TE1[(rk[1] >> 24)] & 0xff] ^
        TD1[TE1[(rk[1] >> 16) & 0xff] & 0xff] ^
        TD2[TE1[(rk[1] >> 8) & 0xff] & 0xff] ^
        TD3[TE1[(rk[1]) & 0xff] & 0xff];
      rk[2] =
        TD0[TE1[(rk[2] >> 24)] & 0xff] ^
        TD1[TE1[(rk[2] >> 16) & 0xff] & 0xff] ^
        TD2[TE1[(rk[2] >> 8) & 0xff] & 0xff] ^
        TD3[TE1[(rk[2]) & 0xff] & 0xff];
      rk[3] =
        TD0[TE1[(rk[3] >> 24)] & 0xff] ^
        TD1[TE1[(rk[3] >> 16) & 0xff] & 0xff] ^
        TD2[TE1[(rk[3] >> 8) & 0xff] & 0xff] ^
        TD3[TE1[(rk[3]) & 0xff] & 0xff];
    }

  return nr;
}

void rijndaelencrypt(FAR const uint32_t *rk, int nr,
                     FAR const uint8_t *pt, FAR uint8_t *ct)
{
  uint32_t s0;
  uint32_t s1;
  uint32_t s2;
  uint32_t s3;
  uint32_t t0;
  uint32_t t1;
  uint32_t t2;
  uint32_t t3;
#ifndef FULL_UNROLL
  int r;
#endif /* ?FULL_UNROLL */

  /* map byte array block to cipher state
   * and add initial round key:
   */

  s0 = GETU32(pt) ^ rk[0];
  s1 = GETU32(pt + 4) ^ rk[1];
  s2 = GETU32(pt + 8) ^ rk[2];
  s3 = GETU32(pt + 12) ^ rk[3];
#ifdef FULL_UNROLL

  /* round 1: */

  t0 = TE0[s0 >> 24] ^ TE1[(s1 >> 16) & 0xff] ^
        TE2[(s2 >> 8) & 0xff] ^ TE3[s3 & 0xff] ^ rk[4];
  t1 = TE0[s1 >> 24] ^ TE1[(s2 >> 16) & 0xff] ^
        TE2[(s3 >> 8) & 0xff] ^ TE3[s0 & 0xff] ^ rk[5];
  t2 = TE0[s2 >> 24] ^ TE1[(s3 >> 16) & 0xff] ^
        TE2[(s0 >> 8) & 0xff] ^ TE3[s1 & 0xff] ^ rk[6];
  t3 = TE0[s3 >> 24] ^ TE1[(s0 >> 16) & 0xff] ^
        TE2[(s1 >> 8) & 0xff] ^ TE3[s2 & 0xff] ^ rk[7];

  /* round 2: */

  s0 = TE0[t0 >> 24] ^ TE1[(t1 >> 16) & 0xff] ^
        TE2[(t2 >> 8) & 0xff] ^ TE3[t3 & 0xff] ^ rk[8];
  s1 = TE0[t1 >> 24] ^ TE1[(t2 >> 16) & 0xff] ^
        TE2[(t3 >> 8) & 0xff] ^ TE3[t0 & 0xff] ^ rk[9];
  s2 = TE0[t2 >> 24] ^ TE1[(t3 >> 16) & 0xff] ^
        TE2[(t0 >> 8) & 0xff] ^ TE3[t1 & 0xff] ^ rk[10];
  s3 = TE0[t3 >> 24] ^ TE1[(t0 >> 16) & 0xff] ^
        TE2[(t1 >> 8) & 0xff] ^ TE3[t2 & 0xff] ^ rk[11];

  /* round 3: */

  t0 = TE0[s0 >> 24] ^ TE1[(s1 >> 16) & 0xff] ^
        TE2[(s2 >> 8) & 0xff] ^ TE3[s3 & 0xff] ^ rk[12];
  t1 = TE0[s1 >> 24] ^ TE1[(s2 >> 16) & 0xff] ^
        TE2[(s3 >> 8) & 0xff] ^ TE3[s0 & 0xff] ^ rk[13];
  t2 = TE0[s2 >> 24] ^ TE1[(s3 >> 16) & 0xff] ^
        TE2[(s0 >> 8) & 0xff] ^ TE3[s1 & 0xff] ^ rk[14];
  t3 = TE0[s3 >> 24] ^ TE1[(s0 >> 16) & 0xff] ^
        TE2[(s1 >> 8) & 0xff] ^ TE3[s2 & 0xff] ^ rk[15];

  /* round 4: */

  s0 = TE0[t0 >> 24] ^ TE1[(t1 >> 16) & 0xff] ^
        TE2[(t2 >> 8) & 0xff] ^ TE3[t3 & 0xff] ^ rk[16];
  s1 = TE0[t1 >> 24] ^ TE1[(t2 >> 16) & 0xff] ^
        TE2[(t3 >> 8) & 0xff] ^ TE3[t0 & 0xff] ^ rk[17];
  s2 = TE0[t2 >> 24] ^ TE1[(t3 >> 16) & 0xff] ^
        TE2[(t0 >> 8) & 0xff] ^ TE3[t1 & 0xff] ^ rk[18];
  s3 = TE0[t3 >> 24] ^ TE1[(t0 >> 16) & 0xff] ^
        TE2[(t1 >> 8) & 0xff] ^ TE3[t2 & 0xff] ^ rk[19];

  /* round 5: */

  t0 = TE0[s0 >> 24] ^ TE1[(s1 >> 16) & 0xff] ^
        TE2[(s2 >> 8) & 0xff] ^ TE3[s3 & 0xff] ^ rk[20];
  t1 = TE0[s1 >> 24] ^ TE1[(s2 >> 16) & 0xff] ^
        TE2[(s3 >> 8) & 0xff] ^ TE3[s0 & 0xff] ^ rk[21];
  t2 = TE0[s2 >> 24] ^ TE1[(s3 >> 16) & 0xff] ^
        TE2[(s0 >> 8) & 0xff] ^ TE3[s1 & 0xff] ^ rk[22];
  t3 = TE0[s3 >> 24] ^ TE1[(s0 >> 16) & 0xff] ^
        TE2[(s1 >> 8) & 0xff] ^ TE3[s2 & 0xff] ^ rk[23];

  /* round 6: */

  s0 = TE0[t0 >> 24] ^ TE1[(t1 >> 16) & 0xff] ^
        TE2[(t2 >> 8) & 0xff] ^ TE3[t3 & 0xff] ^ rk[24];
  s1 = TE0[t1 >> 24] ^ TE1[(t2 >> 16) & 0xff] ^
        TE2[(t3 >> 8) & 0xff] ^ TE3[t0 & 0xff] ^ rk[25];
  s2 = TE0[t2 >> 24] ^ TE1[(t3 >> 16) & 0xff] ^
        TE2[(t0 >> 8) & 0xff] ^ TE3[t1 & 0xff] ^ rk[26];
  s3 = TE0[t3 >> 24] ^ TE1[(t0 >> 16) & 0xff] ^
        TE2[(t1 >> 8) & 0xff] ^ TE3[t2 & 0xff] ^ rk[27];

  /* round 7: */

  t0 = TE0[s0 >> 24] ^ TE1[(s1 >> 16) & 0xff] ^
        TE2[(s2 >> 8) & 0xff] ^ TE3[s3 & 0xff] ^ rk[28];
  t1 = TE0[s1 >> 24] ^ TE1[(s2 >> 16) & 0xff] ^
        TE2[(s3 >> 8) & 0xff] ^ TE3[s0 & 0xff] ^ rk[29];
  t2 = TE0[s2 >> 24] ^ TE1[(s3 >> 16) & 0xff] ^
        TE2[(s0 >> 8) & 0xff] ^ TE3[s1 & 0xff] ^ rk[30];
  t3 = TE0[s3 >> 24] ^ TE1[(s0 >> 16) & 0xff] ^
        TE2[(s1 >> 8) & 0xff] ^ TE3[s2 & 0xff] ^ rk[31];

  /* round 8: */

  s0 = TE0[t0 >> 24] ^ TE1[(t1 >> 16) & 0xff] ^
        TE2[(t2 >> 8) & 0xff] ^ TE3[t3 & 0xff] ^ rk[32];
  s1 = TE0[t1 >> 24] ^ TE1[(t2 >> 16) & 0xff] ^
        TE2[(t3 >> 8) & 0xff] ^ TE3[t0 & 0xff] ^ rk[33];
  s2 = TE0[t2 >> 24] ^ TE1[(t3 >> 16) & 0xff] ^
        TE2[(t0 >> 8) & 0xff] ^ TE3[t1 & 0xff] ^ rk[34];
  s3 = TE0[t3 >> 24] ^ TE1[(t0 >> 16) & 0xff] ^
        TE2[(t1 >> 8) & 0xff] ^ TE3[t2 & 0xff] ^ rk[35];

  /* round 9: */

  t0 = TE0[s0 >> 24] ^ TE1[(s1 >> 16) & 0xff] ^
        TE2[(s2 >> 8) & 0xff] ^ TE3[s3 & 0xff] ^ rk[36];
  t1 = TE0[s1 >> 24] ^ TE1[(s2 >> 16) & 0xff] ^
        TE2[(s3 >> 8) & 0xff] ^ TE3[s0 & 0xff] ^ rk[37];
  t2 = TE0[s2 >> 24] ^ TE1[(s3 >> 16) & 0xff] ^
        TE2[(s0 >> 8) & 0xff] ^ TE3[s1 & 0xff] ^ rk[38];
  t3 = TE0[s3 >> 24] ^ TE1[(s0 >> 16) & 0xff] ^
        TE2[(s1 >> 8) & 0xff] ^ TE3[s2 & 0xff] ^ rk[39];

  if (nr > 10)
    {
      /* round 10: */

      s0 = TE0[t0 >> 24] ^ TE1[(t1 >> 16) & 0xff] ^
            TE2[(t2 >> 8) & 0xff] ^ TE3[t3 & 0xff] ^ rk[40];
      s1 = TE0[t1 >> 24] ^ TE1[(t2 >> 16) & 0xff] ^
            TE2[(t3 >> 8) & 0xff] ^ TE3[t0 & 0xff] ^ rk[41];
      s2 = TE0[t2 >> 24] ^ TE1[(t3 >> 16) & 0xff] ^
            TE2[(t0 >> 8) & 0xff] ^ TE3[t1 & 0xff] ^ rk[42];
      s3 = TE0[t3 >> 24] ^ TE1[(t0 >> 16) & 0xff] ^
            TE2[(t1 >> 8) & 0xff] ^ TE3[t2 & 0xff] ^ rk[43];

      /* round 11: */

      t0 = TE0[s0 >> 24] ^ TE1[(s1 >> 16) & 0xff] ^
            TE2[(s2 >> 8) & 0xff] ^ TE3[s3 & 0xff] ^ rk[44];
      t1 = TE0[s1 >> 24] ^ TE1[(s2 >> 16) & 0xff] ^
            TE2[(s3 >> 8) & 0xff] ^ TE3[s0 & 0xff] ^ rk[45];
      t2 = TE0[s2 >> 24] ^ TE1[(s3 >> 16) & 0xff] ^
            TE2[(s0 >> 8) & 0xff] ^ TE3[s1 & 0xff] ^ rk[46];
      t3 = TE0[s3 >> 24] ^ TE1[(s0 >> 16) & 0xff] ^
            TE2[(s1 >> 8) & 0xff] ^ TE3[s2 & 0xff] ^ rk[47];
      if (nr > 12)
        {
          /* round 12: */

          s0 = TE0[t0 >> 24] ^ TE1[(t1 >> 16) & 0xff] ^
                TE2[(t2 >> 8) & 0xff] ^ TE3[t3 & 0xff] ^ rk[48];
          s1 = TE0[t1 >> 24] ^ TE1[(t2 >> 16) & 0xff] ^
                TE2[(t3 >> 8) & 0xff] ^ TE3[t0 & 0xff] ^ rk[49];
          s2 = TE0[t2 >> 24] ^ TE1[(t3 >> 16) & 0xff] ^
                TE2[(t0 >> 8) & 0xff] ^ TE3[t1 & 0xff] ^ rk[50];
          s3 = TE0[t3 >> 24] ^ TE1[(t0 >> 16) & 0xff] ^
                TE2[(t1 >> 8) & 0xff] ^ TE3[t2 & 0xff] ^ rk[51];

          /* round 13: */

          t0 = TE0[s0 >> 24] ^ TE1[(s1 >> 16) & 0xff] ^
                TE2[(s2 >> 8) & 0xff] ^ TE3[s3 & 0xff] ^ rk[52];
          t1 = TE0[s1 >> 24] ^ TE1[(s2 >> 16) & 0xff] ^
                TE2[(s3 >> 8) & 0xff] ^ TE3[s0 & 0xff] ^ rk[53];
          t2 = TE0[s2 >> 24] ^ TE1[(s3 >> 16) & 0xff] ^
                TE2[(s0 >> 8) & 0xff] ^ TE3[s1 & 0xff] ^ rk[54];
          t3 = TE0[s3 >> 24] ^ TE1[(s0 >> 16) & 0xff] ^
                TE2[(s1 >> 8) & 0xff] ^ TE3[s2 & 0xff] ^ rk[55];
        }
    }

  rk += nr << 2;
#else  /* !FULL_UNROLL */
  /* nr - 1 full rounds: */

  r = nr >> 1;
  for (; ; )
    {
      t0 =
          TE0[(s0 >> 24)] ^
          TE1[(s1 >> 16) & 0xff] ^
          TE2[(s2 >>  8) & 0xff] ^
          TE3[(s3) & 0xff] ^
          rk[4];
      t1 =
          TE0[(s1 >> 24)] ^
          TE1[(s2 >> 16) & 0xff] ^
          TE2[(s3 >>  8) & 0xff] ^
          TE3[(s0) & 0xff] ^
          rk[5];
      t2 =
          TE0[(s2 >> 24)] ^
          TE1[(s3 >> 16) & 0xff] ^
          TE2[(s0 >> 8) & 0xff] ^
          TE3[(s1) & 0xff] ^
          rk[6];
      t3 =
          TE0[(s3 >> 24)] ^
          TE1[(s0 >> 16) & 0xff] ^
          TE2[(s1 >> 8) & 0xff] ^
          TE3[(s2) & 0xff] ^
          rk[7];

      rk += 8;
      if (--r == 0)
        {
            break;
        }

      s0 =
          TE0[(t0 >> 24)] ^
          TE1[(t1 >> 16) & 0xff] ^
          TE2[(t2 >> 8) & 0xff] ^
          TE3[(t3) & 0xff] ^
          rk[0];
      s1 =
          TE0[(t1 >> 24)] ^
          TE1[(t2 >> 16) & 0xff] ^
          TE2[(t3 >> 8) & 0xff] ^
          TE3[(t0) & 0xff] ^
          rk[1];
      s2 =
          TE0[(t2 >> 24)] ^
          TE1[(t3 >> 16) & 0xff] ^
          TE2[(t0 >> 8) & 0xff] ^
          TE3[(t1) & 0xff] ^
          rk[2];
      s3 =
          TE0[(t3 >> 24)] ^
          TE1[(t0 >> 16) & 0xff] ^
          TE2[(t1 >> 8) & 0xff] ^
          TE3[(t2) & 0xff] ^
          rk[3];
    }

#endif /* ?FULL_UNROLL */
  /* apply last round and
   * map cipher state to byte array block:
   */

  s0 =
    (TE2[(t0 >> 24)] & 0xff000000) ^
    (TE3[(t1 >> 16) & 0xff] & 0x00ff0000) ^
    (TE0[(t2 >> 8) & 0xff] & 0x0000ff00) ^
    (TE1[(t3) & 0xff] & 0x000000ff) ^
    rk[0];
  PUTU32(ct, s0);
  s1 =
    (TE2[(t1 >> 24)] & 0xff000000) ^
    (TE3[(t2 >> 16) & 0xff] & 0x00ff0000) ^
    (TE0[(t3 >> 8) & 0xff] & 0x0000ff00) ^
    (TE1[(t0) & 0xff] & 0x000000ff) ^
    rk[1];
  PUTU32(ct + 4, s1);
  s2 =
    (TE2[(t2 >> 24)] & 0xff000000) ^
    (TE3[(t3 >> 16) & 0xff] & 0x00ff0000) ^
    (TE0[(t0 >> 8) & 0xff] & 0x0000ff00) ^
    (TE1[(t1) & 0xff] & 0x000000ff) ^
    rk[2];
  PUTU32(ct + 8, s2);
  s3 =
    (TE2[(t3 >> 24)] & 0xff000000) ^
    (TE3[(t0 >> 16) & 0xff] & 0x00ff0000) ^
    (TE0[(t1 >> 8) & 0xff] & 0x0000ff00) ^
    (TE1[(t2) & 0xff] & 0x000000ff) ^
    rk[3];
  PUTU32(ct + 12, s3);
}

static void rijndaeldecrypt(FAR const uint32_t *rk,
                            int nr, FAR const uint8_t *ct,
                            FAR uint8_t *pt)
{
  uint32_t s0;
  uint32_t s1;
  uint32_t s2;
  uint32_t s3;
  uint32_t t0;
  uint32_t t1;
  uint32_t t2;
  uint32_t t3;
#ifndef FULL_UNROLL
  int r;
#endif /* ?FULL_UNROLL */

  /* map byte array block to cipher state
   * and add initial round key:
   */

    s0 = GETU32(ct) ^ rk[0];
    s1 = GETU32(ct + 4) ^ rk[1];
    s2 = GETU32(ct + 8) ^ rk[2];
    s3 = GETU32(ct + 12) ^ rk[3];
#ifdef FULL_UNROLL

  /* round 1: */

  t0 = TD0[s0 >> 24] ^ TD1[(s3 >> 16) & 0xff] ^
       TD2[(s2 >> 8) & 0xff] ^ TD3[s1 & 0xff] ^ rk[4];
  t1 = TD0[s1 >> 24] ^ TD1[(s0 >> 16) & 0xff] ^
       TD2[(s3 >> 8) & 0xff] ^ TD3[s2 & 0xff] ^ rk[5];
  t2 = TD0[s2 >> 24] ^ TD1[(s1 >> 16) & 0xff] ^
       TD2[(s0 >> 8) & 0xff] ^ TD3[s3 & 0xff] ^ rk[6];
  t3 = TD0[s3 >> 24] ^ TD1[(s2 >> 16) & 0xff] ^
       TD2[(s1 >> 8) & 0xff] ^ TD3[s0 & 0xff] ^ rk[7];

  /* round 2: */

  s0 = TD0[t0 >> 24] ^ TD1[(t3 >> 16) & 0xff] ^
       TD2[(t2 >> 8) & 0xff] ^ TD3[t1 & 0xff] ^ rk[8];
  s1 = TD0[t1 >> 24] ^ TD1[(t0 >> 16) & 0xff] ^
       TD2[(t3 >> 8) & 0xff] ^ TD3[t2 & 0xff] ^ rk[9];
  s2 = TD0[t2 >> 24] ^ TD1[(t1 >> 16) & 0xff] ^
       TD2[(t0 >> 8) & 0xff] ^ TD3[t3 & 0xff] ^ rk[10];
  s3 = TD0[t3 >> 24] ^ TD1[(t2 >> 16) & 0xff] ^
       TD2[(t1 >> 8) & 0xff] ^ TD3[t0 & 0xff] ^ rk[11];

  /* round 3: */

  t0 = TD0[s0 >> 24] ^ TD1[(s3 >> 16) & 0xff] ^
       TD2[(s2 >> 8) & 0xff] ^ TD3[s1 & 0xff] ^ rk[12];
  t1 = TD0[s1 >> 24] ^ TD1[(s0 >> 16) & 0xff] ^
       TD2[(s3 >> 8) & 0xff] ^ TD3[s2 & 0xff] ^ rk[13];
  t2 = TD0[s2 >> 24] ^ TD1[(s1 >> 16) & 0xff] ^
       TD2[(s0 >> 8) & 0xff] ^ TD3[s3 & 0xff] ^ rk[14];
  t3 = TD0[s3 >> 24] ^ TD1[(s2 >> 16) & 0xff] ^
       TD2[(s1 >> 8) & 0xff] ^ TD3[s0 & 0xff] ^ rk[15];

  /* round 4: */

  s0 = TD0[t0 >> 24] ^ TD1[(t3 >> 16) & 0xff] ^
       TD2[(t2 >> 8) & 0xff] ^ TD3[t1 & 0xff] ^ rk[16];
  s1 = TD0[t1 >> 24] ^ TD1[(t0 >> 16) & 0xff] ^
       TD2[(t3 >> 8) & 0xff] ^ TD3[t2 & 0xff] ^ rk[17];
  s2 = TD0[t2 >> 24] ^ TD1[(t1 >> 16) & 0xff] ^
       TD2[(t0 >> 8) & 0xff] ^ TD3[t3 & 0xff] ^ rk[18];
  s3 = TD0[t3 >> 24] ^ TD1[(t2 >> 16) & 0xff] ^
       TD2[(t1 >> 8) & 0xff] ^ TD3[t0 & 0xff] ^ rk[19];

  /* round 5: */

  t0 = TD0[s0 >> 24] ^ TD1[(s3 >> 16) & 0xff] ^
       TD2[(s2 >> 8) & 0xff] ^ TD3[s1 & 0xff] ^ rk[20];
  t1 = TD0[s1 >> 24] ^ TD1[(s0 >> 16) & 0xff] ^
       TD2[(s3 >> 8) & 0xff] ^ TD3[s2 & 0xff] ^ rk[21];
  t2 = TD0[s2 >> 24] ^ TD1[(s1 >> 16) & 0xff] ^
       TD2[(s0 >> 8) & 0xff] ^ TD3[s3 & 0xff] ^ rk[22];
  t3 = TD0[s3 >> 24] ^ TD1[(s2 >> 16) & 0xff] ^
       TD2[(s1 >> 8) & 0xff] ^ TD3[s0 & 0xff] ^ rk[23];

  /* round 6: */

  s0 = TD0[t0 >> 24] ^ TD1[(t3 >> 16) & 0xff] ^
       TD2[(t2 >> 8) & 0xff] ^ TD3[t1 & 0xff] ^ rk[24];
  s1 = TD0[t1 >> 24] ^ TD1[(t0 >> 16) & 0xff] ^
       TD2[(t3 >> 8) & 0xff] ^ TD3[t2 & 0xff] ^ rk[25];
  s2 = TD0[t2 >> 24] ^ TD1[(t1 >> 16) & 0xff] ^
       TD2[(t0 >> 8) & 0xff] ^ TD3[t3 & 0xff] ^ rk[26];
  s3 = TD0[t3 >> 24] ^ TD1[(t2 >> 16) & 0xff] ^
       TD2[(t1 >> 8) & 0xff] ^ TD3[t0 & 0xff] ^ rk[27];

  /* round 7: */

  t0 = TD0[s0 >> 24] ^ TD1[(s3 >> 16) & 0xff] ^
       TD2[(s2 >> 8) & 0xff] ^ TD3[s1 & 0xff] ^ rk[28];
  t1 = TD0[s1 >> 24] ^ TD1[(s0 >> 16) & 0xff] ^
       TD2[(s3 >> 8) & 0xff] ^ TD3[s2 & 0xff] ^ rk[29];
  t2 = TD0[s2 >> 24] ^ TD1[(s1 >> 16) & 0xff] ^
       TD2[(s0 >> 8) & 0xff] ^ TD3[s3 & 0xff] ^ rk[30];
  t3 = TD0[s3 >> 24] ^ TD1[(s2 >> 16) & 0xff] ^
       TD2[(s1 >> 8) & 0xff] ^ TD3[s0 & 0xff] ^ rk[31];

  /* round 8: */

  s0 = TD0[t0 >> 24] ^ TD1[(t3 >> 16) & 0xff] ^
       TD2[(t2 >> 8) & 0xff] ^ TD3[t1 & 0xff] ^ rk[32];
  s1 = TD0[t1 >> 24] ^ TD1[(t0 >> 16) & 0xff] ^
       TD2[(t3 >> 8) & 0xff] ^ TD3[t2 & 0xff] ^ rk[33];
  s2 = TD0[t2 >> 24] ^ TD1[(t1 >> 16) & 0xff] ^
       TD2[(t0 >> 8) & 0xff] ^ TD3[t3 & 0xff] ^ rk[34];
  s3 = TD0[t3 >> 24] ^ TD1[(t2 >> 16) & 0xff] ^
       TD2[(t1 >> 8) & 0xff] ^ TD3[t0 & 0xff] ^ rk[35];

  /* round 9: */

  t0 = TD0[s0 >> 24] ^ TD1[(s3 >> 16) & 0xff] ^
       TD2[(s2 >> 8) & 0xff] ^ TD3[s1 & 0xff] ^ rk[36];
  t1 = TD0[s1 >> 24] ^ TD1[(s0 >> 16) & 0xff] ^
       TD2[(s3 >> 8) & 0xff] ^ TD3[s2 & 0xff] ^ rk[37];
  t2 = TD0[s2 >> 24] ^ TD1[(s1 >> 16) & 0xff] ^
       TD2[(s0 >> 8) & 0xff] ^ TD3[s3 & 0xff] ^ rk[38];
  t3 = TD0[s3 >> 24] ^ TD1[(s2 >> 16) & 0xff] ^
       TD2[(s1 >> 8) & 0xff] ^ TD3[s0 & 0xff] ^ rk[39];
  if (nr > 10)
    {
      /* round 10: */

      s0 = TD0[t0 >> 24] ^ TD1[(t3 >> 16) & 0xff] ^
           TD2[(t2 >> 8) & 0xff] ^ TD3[t1 & 0xff] ^ rk[40];
      s1 = TD0[t1 >> 24] ^ TD1[(t0 >> 16) & 0xff] ^
           TD2[(t3 >> 8) & 0xff] ^ TD3[t2 & 0xff] ^ rk[41];
      s2 = TD0[t2 >> 24] ^ TD1[(t1 >> 16) & 0xff] ^
           TD2[(t0 >> 8) & 0xff] ^ TD3[t3 & 0xff] ^ rk[42];
      s3 = TD0[t3 >> 24] ^ TD1[(t2 >> 16) & 0xff] ^
           TD2[(t1 >> 8) & 0xff] ^ TD3[t0 & 0xff] ^ rk[43];

      /* round 11: */

      t0 = TD0[s0 >> 24] ^ TD1[(s3 >> 16) & 0xff] ^
           TD2[(s2 >> 8) & 0xff] ^ TD3[s1 & 0xff] ^ rk[44];
      t1 = TD0[s1 >> 24] ^ TD1[(s0 >> 16) & 0xff] ^
           TD2[(s3 >> 8) & 0xff] ^ TD3[s2 & 0xff] ^ rk[45];
      t2 = TD0[s2 >> 24] ^ TD1[(s1 >> 16) & 0xff] ^
           TD2[(s0 >> 8) & 0xff] ^ TD3[s3 & 0xff] ^ rk[46];
      t3 = TD0[s3 >> 24] ^ TD1[(s2 >> 16) & 0xff] ^
           TD2[(s1 >> 8) & 0xff] ^ TD3[s0 & 0xff] ^ rk[47];

      if (nr > 12)
        {
          /* round 12: */

          s0 = TD0[t0 >> 24] ^ TD1[(t3 >> 16) & 0xff] ^
               TD2[(t2 >> 8) & 0xff] ^ TD3[t1 & 0xff] ^ rk[48];
          s1 = TD0[t1 >> 24] ^ TD1[(t0 >> 16) & 0xff] ^
               TD2[(t3 >> 8) & 0xff] ^ TD3[t2 & 0xff] ^ rk[49];
          s2 = TD0[t2 >> 24] ^ TD1[(t1 >> 16) & 0xff] ^
               TD2[(t0 >> 8) & 0xff] ^ TD3[t3 & 0xff] ^ rk[50];
          s3 = TD0[t3 >> 24] ^ TD1[(t2 >> 16) & 0xff] ^
               TD2[(t1 >> 8) & 0xff] ^ TD3[t0 & 0xff] ^ rk[51];

          /* round 13: */

          t0 = TD0[s0 >> 24] ^ TD1[(s3 >> 16) & 0xff] ^
               TD2[(s2 >> 8) & 0xff] ^ TD3[s1 & 0xff] ^ rk[52];
          t1 = TD0[s1 >> 24] ^ TD1[(s0 >> 16) & 0xff] ^
               TD2[(s3 >> 8) & 0xff] ^ TD3[s2 & 0xff] ^ rk[53];
          t2 = TD0[s2 >> 24] ^ TD1[(s1 >> 16) & 0xff] ^
               TD2[(s0 >> 8) & 0xff] ^ TD3[s3 & 0xff] ^ rk[54];
          t3 = TD0[s3 >> 24] ^ TD1[(s2 >> 16) & 0xff] ^
               TD2[(s1 >> 8) & 0xff] ^ TD3[s0 & 0xff] ^ rk[55];
        }
    }

  rk += nr << 2;
#else  /* !FULL_UNROLL */
  /* nr - 1 full rounds: */

  r = nr >> 1;
  for (; ; )
    {
      t0 =
          TD0[(s0 >> 24)] ^
          TD1[(s3 >> 16) & 0xff] ^
          TD2[(s2 >> 8) & 0xff] ^
          TD3[(s1) & 0xff] ^
          rk[4];
      t1 =
          TD0[(s1 >> 24)] ^
          TD1[(s0 >> 16) & 0xff] ^
          TD2[(s3 >> 8) & 0xff] ^
          TD3[(s2) & 0xff] ^
          rk[5];
      t2 =
          TD0[(s2 >> 24)] ^
          TD1[(s1 >> 16) & 0xff] ^
          TD2[(s0 >> 8) & 0xff] ^
          TD3[(s3) & 0xff] ^
          rk[6];
      t3 =
          TD0[(s3 >> 24)] ^
          TD1[(s2 >> 16) & 0xff] ^
          TD2[(s1 >> 8) & 0xff] ^
          TD3[(s0) & 0xff] ^
          rk[7];

      rk += 8;
      if (--r == 0)
        {
          break;
        }

      s0 =
          TD0[(t0 >> 24)] ^
          TD1[(t3 >> 16) & 0xff] ^
          TD2[(t2 >> 8) & 0xff] ^
          TD3[(t1) & 0xff] ^
          rk[0];
      s1 =
          TD0[(t1 >> 24)] ^
          TD1[(t0 >> 16) & 0xff] ^
          TD2[(t3 >> 8) & 0xff] ^
          TD3[(t2) & 0xff] ^
          rk[1];
      s2 =
          TD0[(t2 >> 24)] ^
          TD1[(t1 >> 16) & 0xff] ^
          TD2[(t0 >> 8) & 0xff] ^
          TD3[(t3) & 0xff] ^
          rk[2];
      s3 =
          TD0[(t3 >> 24)] ^
          TD1[(t2 >> 16) & 0xff] ^
          TD2[(t1 >> 8) & 0xff] ^
          TD3[(t0) & 0xff] ^
          rk[3];
    }

#endif /* ?FULL_UNROLL */
  /* apply last round and
   * map cipher state to byte array block:
   */

  s0 =
    (TD4[(t0 >> 24)] << 24) ^
    (TD4[(t3 >> 16) & 0xff] << 16) ^
    (TD4[(t2 >> 8) & 0xff] << 8) ^
    (TD4[(t1) & 0xff]) ^
    rk[0];
  PUTU32(pt, s0);
  s1 =
    (TD4[(t1 >> 24)] << 24) ^
    (TD4[(t0 >> 16) & 0xff] << 16) ^
    (TD4[(t3 >> 8) & 0xff] << 8) ^
    (TD4[(t2) & 0xff]) ^
    rk[1];
  PUTU32(pt +  4, s1);
  s2 =
    (TD4[(t2 >> 24)] << 24) ^
    (TD4[(t1 >> 16) & 0xff] << 16) ^
    (TD4[(t0 >> 8) & 0xff] << 8) ^
    (TD4[(t3) & 0xff]) ^
    rk[2];
  PUTU32(pt +  8, s2);
  s3 =
    (TD4[(t3 >> 24)] << 24) ^
    (TD4[(t2 >> 16) & 0xff] << 16) ^
    (TD4[(t1 >> 8) & 0xff] << 8) ^
    (TD4[(t0) & 0xff]) ^
    rk[3];
  PUTU32(pt + 12, s3);
}

/* setup key context for encryption only */

int rijndael_set_key_enc_only(FAR rijndael_ctx *ctx,
                              FAR const u_char *key,
                              int bits)
{
  int rounds;

  rounds = rijndael_key_setup_enc(ctx->ek, key, bits);
  if (rounds == 0)
    {
      return -1;
    }

  ctx->nr = rounds;
  ctx->enc_only = 1;

  return 0;
}

/* setup key context for both encryption and decryption */

int rijndael_set_key(FAR rijndael_ctx *ctx, FAR const u_char *key, int bits)
{
  int rounds;

  rounds = rijndael_key_setup_enc(ctx->ek, key, bits);
  if (rounds == 0)
    {
      return -1;
    }

  if (rijndael_key_setup_dec(ctx->dk, key, bits) != rounds)
    {
      return -1;
    }

  ctx->nr = rounds;
  ctx->enc_only = 0;

  return 0;
}

void rijndael_decrypt(FAR rijndael_ctx *ctx,
                      FAR const u_char *src,
                      FAR u_char *dst)
{
  rijndaeldecrypt(ctx->dk, ctx->nr, src, dst);
}

void rijndael_encrypt(FAR rijndael_ctx *ctx,
                      FAR const u_char *src,
                      FAR u_char *dst)
{
  rijndaelencrypt(ctx->ek, ctx->nr, src, dst);
}
