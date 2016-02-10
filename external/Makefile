.PHONY: clean

default: m2html genpath_exclude/genpath_exclude.m

m2html: m2html.zip
	unzip $< &&	chown -R $(shell whoami) $@ && chmod -R +w $@ && touch m2html

clean:
	rm -rf m2html
	rm -rf genpath_exclude

genpath_exclude/genpath_exclude.m:                                                                    
	if [ ! -d genpath_exclude ]; then \
	mkdir genpath_exclude && \
	cd genpath_exclude && \
	unzip ../genpath_exclude.zip && \
	touch genpath_exclude.m; \
	fi
