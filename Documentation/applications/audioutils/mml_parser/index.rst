========================================================
``mml_parser`` Music Macro Language (MML) Parser library
========================================================

MML has often been used as a language for describing music in strings, for example,
in the BASIC language. The mml_parser is a minimalistic Music Macro Language parser
library written in pure C intended for resource-constrained platforms, especially
microcontrollers and other embedded systems.

Supported Syntax on this library
--------------------------------

Notes
~~~~~

* **C** (Do)
* **D** (Re)
* **E** (Mi)
* **F** (Fa)
* **G** (Sol)
* **A** (La)
* **B** (Ti)

Sharp and Flat
~~~~~~~~~~~~~~

Add "#" or "+" after the note indicates Sharp. ex: ``"C#"`` ``"C+."``
Add "-" after the note indicates Flat. ex: ``"C-"``

Length
~~~~~~

Length of the tone can be specified in two ways.
One is to specify a length for each note. Add a number after the note in 1, 2, 4, 8, 16, 32 or 64.
ex: ``"C8 C16 C#4."``

The other is to use ``L`` . The ``L`` sets default length. If the note without length, the number which is
indicated by the "L" is used.  The number followed after the "L" can be in 1, 2, 4, 8, 16, 32 or 64.
ex: ``"L4 A"`` means "A" with length 4.

In addition, dot is supported. For example, length of "C4." is 4 + 8. Length of "C16.." is 4 + 8 + 16.

Rest
~~~~

Rest is represented by "R".
Length of the rest is following after the "R" as the same as the note length.  If no length is
specified, the length specified by the ”L" is used.

Chord
~~~~~

Chord is supported. If some notes are enclosed in parentheses by ``[`` and ``]``, they are interpreted as
a chord. ex: ``"[CEG]"`` is a chord of Do, Mi and Sol.
Chord's length can be put after ``]`` . ex: ``"[CEG]4"`` is a chord with 4 length.

Note: Max notes in a chord is defined as MAX_CHORD_NOTES in ``mml_parser.h``.

Tuplet
~~~~~~

Tuplet is supported. If some notes and Chord are enclosed in parentheses by ``{`` and ``}``, they are
interpreted as a tuplet. ex: ``"{C E G [CEG]}"`` is a tuplet with C, E, G and chord of CEG.
Tuplet's length can be put after "}", and the length is divided equally among each note.
ex: in ``"{C E G [CEG]}4"`` case, C, E, G and chord CEG has each a quarter of the L4 length.

Octave
~~~~~~

Octarve is controlled by "O", ">" or "<".
When "O" is used, the O is followed by a number indicating the octave.
When ">" is used, the value of the new octave is the current octave plus one.
When "<" is used, the value of the new octave is the current octave minus one.
ex: ``"CDEFGAB > C R C < BAGFEDC"``, ``"O4 CDEFGAB O5 C R C O4 BAGFEDC"``

Tempo
~~~~~

Tempo is indicated as "T" and numter following after the "T".
Tempo number decide a speed of the score. This value is used for culculating sample number for
the note (or rest).
ex: ``"T120"``

Volume
~~~~~~

Volume can be controlled by "V". And numter following after the "V".
ex: ``"V4"``

Example of a score
------------------

The beginning of the Do Re Mi Song
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Tempo 120, Voulume 10, Octave 4, Default length is 4.

``"T120 V10 O4 L4 C. D8 E. C8 E C E2 D. E8 {FF} {ED} F2"``

Provided C Functions
--------------------

mml_parser is providing just 2 functions.

init_mml()
~~~~~~~~~~

Initialize an instance of mml parser.

.. code-block:: c

   #include <audioutils/mml_parser.h>

   int init_mml(FAR struct music_macro_lang_s *mml,
                int fs, int tempo, int octave, int length);

The function initializes ``struct music_macro_lang_s`` instance provided as 1st argument.
The argument ``fs`` is a sampling frequency of target audio output system, and this value is used for
calculating sample number in case of a tempo and length of note.
``tempo``, ``octave`` and ``length`` specify initial values for tempo, octave, and length, respectively.

On success, init_mml() returns 0. On error, it returns an negative value.

Currently no error is happened.

parse_mml()
~~~~~~~~~~~

Parse MML from given string.

.. code-block:: c

   #include <audioutils/mml_parser.h>

   int parse_mml(FAR struct music_macro_lang_s *mml,
                FAR char **score, FAR struct mml_result_s *result);

parse_mml() parses the first MML of the string given by the argument ``score`` and gives
the result in the return value and the argument ``result``.
The ``result`` is an instance of mml_result_s, which contains note_idx, length, and
chord_notes as members. The meaning of the value of each member depends on the return value.

On error, a nevative value is returned.
On success, following values can be returned. And those values are defined in ``mml_parser.h``.

===================== ===========================================
Return values          Description
===================== ===========================================

MML_TYPE_EOF          This means that it have reached the end of the string. The content of the ``result`` has no meaning.
MML_TYPE_NOTE         This indicates that some note has been parsed. The scale of the note is stored in ``note_idx[0]``. The length of the note is given by the ``length`` member as the number of samples. In the case of tuplet, this return value is returned at the time each note is parsed. In other words, a tuplet is parsed as a single note.
MML_TYPE_REST         This indicates the ``rest`` has been parsed. The length of it is given by the ``length`` member as the number of samples.
MML_TYPE_TEMPO        This indicates ``"T"`` is parsed. ``length`` member in ``result`` has the value of the tempo. But tempo value is kept in mml instance for calculating sample number for each notes. So basically, no need to handle this return value in your code.
MML_TYPE_LENGTH       This indicates ``"L"`` is parsed. ``length`` member in ``result`` has the value of the parsed length. But current length value is kept in mml instance. So basically, no need to handle this return value in your code. |
MML_TYPE_OCTAVE       This indicates ``"O"``, ``">"``, or ``"<"`` is parsed. ``length`` member in ``result`` has the value of the octave. But the octave is encoded in ``note_idx`` in ``MML_TYPE_NOTE`` case. So basically, no need to handle this return value in your code.
MML_TYPE_TUPLETSTART  This indicates tuplet is just started. And total length of the tuplet is stored in ``length`` of ``result`` members.
MML_TYPE_TUPLETDONE   This indicates the tuplet is just finished.
MML_TYPE_VOLUME       This indicates ``"V"`` is parsed. ``length`` member in ``result`` has the value of the parsed volume.
MML_TYPE_TONE         T.B.D.
MML_TYPE_CHORD        This indicates a chord is parsed. Chord has some notes, and how many notes is stored in ``chord_notes`` member of ``result``. And each notes are stored in ``note_idx[]``. Length of the chord is stored in ``length`` member of ``result``.
===================== ===========================================

The value of ``note_idx[]`` is encoding octave, like

====== ==== ==============
Octave Note node_idx value
====== ==== ==============
O0     C    0 
O0     C#   1 
O0     D    2 
O0     D#   3 
O0     E    4 
O0     F    5 
O0     F#   6 
O0     G    7 
O0     G#   8 
O0     A    9 
O0     A#   10
O0     B    11
O1     C    12
====== ==== ==============

And so on.

So for example, G# at Octave 4 is encoded as 56.

Following error code can be received as return value.

================================= =================
Error code                        Description
================================= =================
MML_TYPE_NOTE_ERROR           
MML_TYPE_REST_ERROR           
MML_TYPE_TEMPO_ERROR          
MML_TYPE_LENGTH_ERROR         
MML_TYPE_OCTAVE_ERROR         
MML_TYPE_VOLUME_ERROR         
MML_TYPE_TUPLET_ERROR         
MML_TYPE_TONE_ERROR           
MML_TYPE_CHORD_ERROR          
MML_TYPE_ILLIGAL_COMPOSITION  
MML_TYPE_ILLIGAL_TOOMANY_NOTES
MML_TYPE_ILLIGAL_TOOFEW_NOTES 
MML_TYPE_ILLIGAL_DOUBLE_TUPLET
================================= =================

Running unit tests
------------------

Please see examples/mml_parser

Bugs
----

There are plenty. Report them on GitHub, or - even better - open a pull request.
Please write unit tests for any new functions you add - it's fun!

Author
------

mml_parser was written by Takayoshi Koizumi &lt;takayoshi.koizumi@gmail.com&gt;
