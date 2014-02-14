EXTRA_DIST = $(SCRIPT_FILES) $(OTHER_FILES)

SCRIPT_FILES = \
	2c2-a.xml \
	2c6-a.xml \
	2c6-b.xml \
	2c10-a.xml \
	2d2-a.xml

OTHER_FILES = \
	2c2-a_init.xml \
	2c6-a_init.xml \
	2c6-b_init.xml \
	2c10-a_init.xml \
	2d2-a_init.xml

check-local:
	-xmllint --noout --schema $(top_srcdir)/JSBSimScript.xsd $(SCRIPT_FILES)

