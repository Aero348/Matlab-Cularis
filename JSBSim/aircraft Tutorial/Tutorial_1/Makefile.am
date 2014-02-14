EXTRA_DIST = \
	Tutorial_1.xml \
	plots\2c2-a.p \
	plots\2c10-a.p \
	plots\2d2-a.p \
	Results\.empty \
	scripts\2c2-a.xml \
	scripts\2c2-a_init.xml \
	scripts\2c6-a.xml \
	scripts\2c6-a_init.xml \
	scripts\2c6-b.xml \
	scripts\2c6-b_init.xml \
	scripts\2c10-a.xml \
	scripts\2c10-a_init.xml \
	scripts\2d2-a.xml \
	scripts\2d2-a_init.xml \
	INSTALL

SUBDIRS = \
	scripts

# No Makefile.am: Results, plots

aircraft = Tutorial_1

check-local:
	-xmllint --noout --schema $(top_srcdir)/JSBSim.xsd $(aircraft).xml
	$(top_srcdir)/src/JSBsim \
	  --root=$(top_srcdir) \
	  --catalog=$(aircraft) \
	  1>/dev/null

